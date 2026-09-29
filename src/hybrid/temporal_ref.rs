//! CPU reference for temporal accumulation of the indirect-diffuse GI
//! term (see `cpu_ref.rs::blur_indirect_at`'s own doc comment for why a
//! same-frame spatial blur alone isn't enough — this module is the
//! reprojection/rejection/blend half of the fix, kept in its own file
//! rather than growing `cpu_ref.rs` further since that file is already
//! large and this is a genuinely separate concern: `cpu_ref.rs` is about
//! WHAT a pixel sees this frame, this module is about HOW that value
//! combines with what the same surface point looked like last frame).
//!
//! Root cause being fixed: `cpu_ref::indirect_diffuse` evaluates only 5
//! fixed hemisphere samples per pixel per frame — a small enough sample
//! count that even after interleaved-gradient-noise jittering and a
//! same-frame spatial blur (`blur_indirect_at`), the estimator still
//! shows visible banding (a real, confirmed, currently-shipped artifact —
//! see `PROGRESS.md`'s Stage-B-follow-up entry). Temporal accumulation
//! amortizes the effective sample count across frames instead of paying
//! for it all in one frame: each frame's fresh 5-sample estimate is
//! blended with a running average of every prior frame's estimate at the
//! (reprojected) same world-space point, converging toward a much
//! higher effective sample count over a handful of frames — the standard
//! mechanism production real-time GI denoisers (SVGF, ReSTIR) use.
//!
//! Every function here is a faithful-by-construction reference for its
//! `hybrid_temporal.wgsl` mirror, following this project's established
//! CPU-reference-first convention (see `mod.rs`'s own doc comment) — WGSL
//! is written only after these are proven correct with `cargo test`.

use bevy::math::{Mat4, Quat, Vec2, Vec3};

/// Reprojects a CURRENT-frame world-space point to where the same
/// surface point (in the object's own local space) was last frame —
/// step 1 of temporal reprojection (see this module's own doc comment).
///
/// `obj_current`/`obj_previous` are `(translation, rotation)` pairs for
/// whichever object this point belongs to (or `None` for a sky/miss
/// pixel, which is treated as world-space-static — camera motion only,
/// matching how a real motion-vector pass treats a parallax-free/
/// infinitely-distant background). Both must be `Some` or both `None`;
/// mixing them isn't a case this function's caller (the pixel either hit
/// an object or it didn't) can ever produce.
///
/// Math: undo the CURRENT frame's rigid transform to bring the point into
/// the object's own local space (`inverse(rotation_current) * (p -
/// translation_current)`), then reapply the PREVIOUS frame's rigid
/// transform to bring that same local-space point back into world space
/// as it was last frame (`rotation_previous * local + translation_previous`).
/// This is exactly "undo where the object moved to, redo where it used
/// to be" — correct for any rigid (translate+rotate, no scale) motion,
/// which is the only kind of per-object motion this renderer's `Shape`
/// entities undergo (see `extract::object_gpu_from`: only translation and
/// an inverse rotation quaternion are ever baked per object, never a
/// scale).
pub fn reproject_world_point(
    p_world_current: Vec3,
    obj_current: Option<(Vec3, Quat)>,
    obj_previous: Option<(Vec3, Quat)>,
) -> Vec3 {
    match (obj_current, obj_previous) {
        (Some((translation_current, rotation_current)), Some((translation_previous, rotation_previous))) => {
            let local = rotation_current.inverse() * (p_world_current - translation_current);
            rotation_previous * local + translation_previous
        }
        _ => p_world_current,
    }
}

/// Projects a PREVIOUS-frame world-space point through the PREVIOUS
/// camera's `clip_from_world` matrix into a `[0, 1]` UV coordinate — step
/// 2 of temporal reprojection. Returns `None` when the point is behind
/// the previous camera (`w <= 0`, a degenerate/invalid projection no
/// screen-space UV can represent) — callers must treat `None` exactly
/// like an out-of-`[0,1]`-bounds UV in the disocclusion test
/// (`disocclusion_rejected`'s `uv_in_bounds` parameter), since both mean
/// "there is no valid previous-frame screen location to reproject to."
///
/// UV convention: `(0, 0)` = bottom-left, `(1, 1)` = top-right in NDC
/// terms before the final flip — matches this project's own
/// `frag_coord_to_uv`/`uv_to_ndc` usage in `hybrid_trace.wgsl` (Bevy's
/// `bevy_render::view` convention: NDC `y` is flipped relative to
/// texture-space `v` when converting to/from a `frag_coord`-style UV).
/// This function only produces NDC-derived `[0,1]` UV, not a
/// texture-texel-indexed coordinate — callers multiply by viewport size
/// separately, matching how `hybrid_trace.wgsl`'s own `generate_primary_ray`
/// keeps UV and pixel-index math as separate steps.
pub fn world_to_previous_uv(p_world_previous: Vec3, prev_clip_from_world: Mat4) -> Option<Vec2> {
    let clip = prev_clip_from_world * p_world_previous.extend(1.0);
    if clip.w <= 0.0 {
        return None;
    }
    let ndc = clip.truncate() / clip.w;
    // NDC [-1, 1] -> UV [0, 1], flipping Y (NDC +Y is up, UV +Y is down —
    // same flip `uv_to_ndc`'s inverse would apply, see this function's
    // own doc comment).
    Some(Vec2::new(ndc.x * 0.5 + 0.5, 1.0 - (ndc.y * 0.5 + 0.5)))
}

/// How quickly the disocclusion test's relative-depth-difference check
/// rejects history — mirrors `cpu_ref::BLUR_DEPTH_SIGMA`'s own role and
/// magnitude exactly (same "scale by the CENTER pixel's own depth, not
/// an absolute world-unit threshold" reasoning: a given absolute depth
/// gap should read as "probably the same surface" at a shallow-grazing
/// far distance but "probably a different surface" at a close-up
/// silhouette edge — see `cpu_ref::BLUR_DEPTH_SIGMA`'s own doc comment
/// for the full argument, reused verbatim here rather than re-derived).
pub const TEMPORAL_DEPTH_SIGMA: f32 = 0.05;

/// How much a reprojected history normal is allowed to diverge from the
/// current pixel's own normal before history is rejected — mirrors
/// `cpu_ref::BLUR_NORMAL_SIGMA`'s role (see that constant's own doc
/// comment), but used here as a hard cosine-similarity CUTOFF (accept/
/// reject), not a continuous weight exponent — temporal rejection is a
/// binary decision (either this frame's estimate stands alone, or it
/// blends with history), unlike the spatial blur's continuous weighting
/// of many neighbors at once.
pub const TEMPORAL_NORMAL_COS_THRESHOLD: f32 = 0.9;

/// Step 3 of temporal reprojection: decide whether a reprojected history
/// sample is trustworthy enough to blend with this frame's fresh
/// estimate, or must be discarded (falling back to "this frame's raw
/// value only, no history" — exactly `history_length == 0`'s behavior in
/// `temporal_blend`). Four independent rejection reasons, matching the
/// standard SVGF-style disocclusion test: off-screen reprojection, a
/// depth discontinuity (the reprojected point is no longer on the same
/// surface — e.g. an object moved away, revealing background), and a
/// normal discontinuity (the reprojected point is now on a differently-
/// facing surface — e.g. a silhouette/edge pixel).
pub fn disocclusion_rejected(
    current_depth: f32,
    history_depth: f32,
    current_normal: Vec3,
    history_normal: Vec3,
    uv_in_bounds: bool,
) -> bool {
    if !uv_in_bounds {
        return true;
    }
    let depth_scale = TEMPORAL_DEPTH_SIGMA * current_depth.max(1e-4);
    let depth_diff = (current_depth - history_depth).abs();
    if depth_diff > depth_scale {
        return true;
    }
    let normal_similarity = current_normal.normalize().dot(history_normal.normalize());
    if normal_similarity < TEMPORAL_NORMAL_COS_THRESHOLD {
        return true;
    }
    false
}

/// Step 4: the clamped exponential-moving-average blend — the standard
/// SVGF/TAA "running average with a decaying-then-floored blend weight"
/// formula, NOT a fixed-alpha EMA (a fixed alpha never converges to a
/// low-noise steady state at the sample counts this renderer's 5-sample-
/// per-frame estimator needs: early frames must blend heavily toward new
/// data since there's little history yet, while converged frames should
/// blend only a little so accumulated noise reduction isn't thrown away
/// by every new frame's own fresh noise).
///
/// `history_length` is frames of accumulated history so far (`0.0` means
/// "no valid history — this frame's `current` value is used unchanged,
/// exactly", both for a brand-new pixel and for one whose reprojected
/// history was just rejected by `disocclusion_rejected`).
/// `max_history_length` caps how much the blend weight can decay (see
/// `TemporalConfig::max_history_length`'s own doc comment for why an
/// uncapped EMA is wrong for a still-slowly-changing scene). Returns the
/// blended value and the new (possibly incremented, capped) history
/// length for next frame.
pub fn temporal_blend(current: Vec3, history: Vec3, history_length: f32, max_history_length: f32) -> (Vec3, f32) {
    let new_length = (history_length + 1.0).min(max_history_length.max(1.0));
    let alpha = 1.0 / new_length;
    let blended = history.lerp(current, alpha);
    (blended, new_length)
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;

    use super::*;

    // -----------------------------------------------------------------
    // reproject_world_point
    // -----------------------------------------------------------------

    #[test]
    fn reproject_static_object_is_identity() {
        let obj = Some((Vec3::new(1.0, 2.0, 3.0), Quat::from_rotation_y(0.4)));
        let p = Vec3::new(5.0, 1.0, -2.0);
        let reprojected = reproject_world_point(p, obj, obj);
        assert!((reprojected - p).length() < 1e-5, "a static object's own point should reproject to itself exactly");
    }

    #[test]
    fn reproject_translated_object_shifts_by_exactly_the_translation_delta() {
        let current = (Vec3::new(10.0, 0.0, 0.0), Quat::IDENTITY);
        let previous = (Vec3::new(7.0, 0.0, 0.0), Quat::IDENTITY);
        let p = Vec3::new(10.5, 1.0, 0.0);
        let reprojected = reproject_world_point(p, Some(current), Some(previous));
        // The object moved +3 on X between "previous" and "current", so
        // undoing that motion should shift the point by exactly -3 on X.
        assert!((reprojected - Vec3::new(7.5, 1.0, 0.0)).length() < 1e-5, "got {reprojected:?}");
    }

    #[test]
    fn reproject_rotated_object_matches_an_independently_computed_rotation() {
        let current = (Vec3::ZERO, Quat::from_rotation_y(FRAC_PI_2));
        let previous = (Vec3::ZERO, Quat::IDENTITY);
        // A point at local-space (1, 0, 0) rotated by +90 degrees about Y
        // in the CURRENT frame sits at world (0, 0, -1) (right-handed Y
        // rotation convention). Reprojecting to the PREVIOUS frame (no
        // rotation) should recover local space exactly: (1, 0, 0).
        let p_world_current = current.1 * Vec3::new(1.0, 0.0, 0.0);
        let reprojected = reproject_world_point(p_world_current, Some(current), Some(previous));
        assert!((reprojected - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-4, "got {reprojected:?}");
    }

    #[test]
    fn reproject_sky_pixel_with_no_object_is_unchanged() {
        let p = Vec3::new(100.0, 50.0, -30.0);
        let reprojected = reproject_world_point(p, None, None);
        assert_eq!(reprojected, p, "a miss/sky pixel has no object to undo motion for — must pass through unchanged");
    }

    // -----------------------------------------------------------------
    // world_to_previous_uv
    // -----------------------------------------------------------------

    fn simple_view_proj(eye: Vec3, look_dir: Vec3) -> Mat4 {
        let proj = Mat4::perspective_rh(90f32.to_radians(), 1.0, 0.1, 100.0);
        let view = Mat4::look_to_rh(eye, look_dir, Vec3::Y);
        proj * view
    }

    #[test]
    fn a_point_at_the_previous_camera_eye_has_no_valid_reprojection() {
        let clip_from_world = simple_view_proj(Vec3::new(0.0, 0.0, 5.0), Vec3::new(0.0, 0.0, -1.0));
        let at_eye = Vec3::new(0.0, 0.0, 5.0);
        assert_eq!(world_to_previous_uv(at_eye, clip_from_world), None, "a point exactly at the camera has w <= 0");
    }

    #[test]
    fn a_point_dead_center_of_the_previous_view_reprojects_to_uv_half_half() {
        let eye = Vec3::new(0.0, 0.0, 5.0);
        let look_dir = Vec3::new(0.0, 0.0, -1.0);
        let clip_from_world = simple_view_proj(eye, look_dir);
        let straight_ahead = eye + look_dir * 10.0;
        let uv = world_to_previous_uv(straight_ahead, clip_from_world).expect("expected a valid reprojection");
        assert!((uv - Vec2::new(0.5, 0.5)).length() < 1e-4, "got {uv:?}");
    }

    #[test]
    fn camera_translated_right_shifts_reprojected_uv_in_the_expected_direction() {
        // Previous camera sits to the LEFT of a fixed world point (looking
        // down -Z with no yaw), so that point should appear shifted
        // toward the RIGHT half of the previous frame's own view (UV.x >
        // 0.5) — pins the sign convention down in Rust before it becomes
        // a hard-to-debug visual bug once ported to WGSL.
        let eye = Vec3::new(-2.0, 0.0, 5.0);
        let look_dir = Vec3::new(0.0, 0.0, -1.0);
        let clip_from_world = simple_view_proj(eye, look_dir);
        let world_point = Vec3::new(0.0, 0.0, 0.0);
        let uv = world_to_previous_uv(world_point, clip_from_world).expect("expected a valid reprojection");
        assert!(uv.x > 0.5, "expected the point to reproject right-of-center, got {uv:?}");
    }

    // -----------------------------------------------------------------
    // disocclusion_rejected
    // -----------------------------------------------------------------

    #[test]
    fn identical_depth_and_normal_in_bounds_is_accepted() {
        assert!(!disocclusion_rejected(10.0, 10.0, Vec3::Y, Vec3::Y, true));
    }

    #[test]
    fn a_large_relative_depth_difference_is_rejected() {
        // At depth 10 with TEMPORAL_DEPTH_SIGMA = 0.05, the accept window
        // is +/-0.5 — a history depth of 20 (10 units away) is well
        // outside it.
        assert!(disocclusion_rejected(10.0, 20.0, Vec3::Y, Vec3::Y, true));
    }

    #[test]
    fn a_perpendicular_normal_is_rejected() {
        assert!(disocclusion_rejected(10.0, 10.0, Vec3::Y, Vec3::X, true));
    }

    #[test]
    fn an_out_of_bounds_uv_is_rejected_regardless_of_depth_and_normal_agreement() {
        assert!(disocclusion_rejected(10.0, 10.0, Vec3::Y, Vec3::Y, false));
    }

    #[test]
    fn depth_difference_exactly_at_the_threshold_boundary_is_accepted_not_rejected() {
        // depth_scale at depth=10.0 is exactly 0.5; a diff of exactly 0.5
        // must NOT trip the strict `>` rejection check.
        assert!(!disocclusion_rejected(10.0, 10.5, Vec3::Y, Vec3::Y, true));
    }

    // -----------------------------------------------------------------
    // temporal_blend
    // -----------------------------------------------------------------

    #[test]
    fn zero_history_length_passes_through_the_current_value_exactly() {
        let (blended, new_length) = temporal_blend(Vec3::new(1.0, 2.0, 3.0), Vec3::ZERO, 0.0, 24.0);
        assert_eq!(blended, Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(new_length, 1.0);
    }

    #[test]
    fn history_length_at_the_cap_uses_the_minimum_alpha_and_never_exceeds_the_cap() {
        let (_, new_length) = temporal_blend(Vec3::ONE, Vec3::ZERO, 24.0, 24.0);
        assert_eq!(new_length, 24.0, "history length must not grow past its cap");
    }

    #[test]
    fn repeated_blending_of_the_same_value_converges_toward_it() {
        let target = Vec3::new(0.8, 0.3, 0.1);
        let mut history = Vec3::new(-5.0, 5.0, -5.0); // a deliberately far-off starting "history"
        let mut length = 0.0f32;
        for _ in 0..40 {
            let (blended, new_length) = temporal_blend(target, history, length, 24.0);
            history = blended;
            length = new_length;
        }
        assert!(
            (history - target).length() < 0.05,
            "expected convergence toward {target:?} after many blends, got {history:?}"
        );
    }

    #[test]
    fn history_length_grows_by_exactly_one_each_call_until_the_cap() {
        let mut length = 0.0f32;
        let mut history = Vec3::ZERO;
        for expected in 1..=5 {
            let (blended, new_length) = temporal_blend(Vec3::ONE, history, length, 24.0);
            assert_eq!(new_length, expected as f32);
            history = blended;
            length = new_length;
        }
    }

    // -----------------------------------------------------------------
    // Composed two-frame integration test
    // -----------------------------------------------------------------

    #[test]
    fn two_frame_accumulation_moves_closer_to_the_true_average_than_either_raw_frame() {
        // A tiny synthetic scenario: one object, static camera and
        // object (isolates the accumulation math itself from reprojection
        // edge cases, which the dedicated tests above already cover).
        // Frame 1's raw noisy indirect estimate is 0.2 low of the "true"
        // converged value; frame 2's is 0.2 high — a real single-bounce
        // estimator's per-frame noise, exactly what accumulation is meant
        // to average out.
        let true_value = Vec3::new(0.5, 0.5, 0.5);
        let frame_1_raw = true_value - Vec3::splat(0.2);
        let frame_2_raw = true_value + Vec3::splat(0.2);

        let obj = Some((Vec3::ZERO, Quat::IDENTITY));
        let p_world = Vec3::new(0.0, 1.0, 0.0);
        let clip_from_world = simple_view_proj(Vec3::new(0.0, 1.0, 5.0), Vec3::new(0.0, 0.0, -1.0));
        let current_depth = 5.0;
        let current_normal = Vec3::Y;

        // Frame 1: no history yet.
        let (frame_1_accumulated, history_length) = temporal_blend(frame_1_raw, Vec3::ZERO, 0.0, 24.0);
        assert_eq!(frame_1_accumulated, frame_1_raw);

        // Frame 2: reproject (static scene, so this recovers the exact
        // same world point) and confirm history is accepted, then blend.
        let p_reprojected = reproject_world_point(p_world, obj, obj);
        let uv = world_to_previous_uv(p_reprojected, clip_from_world);
        let rejected = match uv {
            Some(_) => disocclusion_rejected(current_depth, current_depth, current_normal, current_normal, true),
            None => true,
        };
        assert!(!rejected, "a static scene's own history must be accepted, not rejected");
        let (frame_2_accumulated, _) = temporal_blend(frame_2_raw, frame_1_accumulated, history_length, 24.0);

        let error_before_any_accumulation = (frame_2_raw - true_value).length();
        let error_after_accumulation = (frame_2_accumulated - true_value).length();
        assert!(
            error_after_accumulation < error_before_any_accumulation,
            "two-frame accumulation ({frame_2_accumulated:?}) should be closer to the true value {true_value:?} \
             than frame 2's own raw noisy estimate ({frame_2_raw:?}) alone"
        );
    }
}
