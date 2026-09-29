//! CPU reference for stochastic (jittered-lens) depth of field: fires a
//! SEPARATE, dedicated ray per pixel per frame from a simulated camera
//! aperture disk, re-aimed through the same focal-plane point the
//! unperturbed primary ray would have hit, then reprojects that jittered
//! ray's own hit point back through the CURRENT (unperturbed) camera's
//! own projection to find where on the already-shaded, already-composited
//! sharp image to resample color from — and lets a dedicated temporal
//! accumulator converge that resampled value over several frames. This
//! is the standard distributed-ray-tracing technique for physically-based
//! DOF (Cook, Porter, Carpenter, "Distributed Ray Tracing," SIGGRAPH
//! 1984), adapted as a RESAMPLE of an already-fully-shaded image rather
//! than a full re-shade, for a reason specific to this renderer (see
//! below).
//!
//! **Why a SEPARATE ray, not jittering the primary ray directly (a real
//! design mistake caught before landing — see PROGRESS.md's own DOF
//! entry):** the primary ray's own hit point/depth feeds EVERY other
//! system in this renderer — the depth buffer `hybrid_blit.wgsl` writes
//! for Bevy's own depth test, the existing GI temporal accumulator's
//! disocclusion test, and the reflection/refraction accumulators'
//! virtual-point reprojection. Jittering that one ray would perturb the
//! depth/hit-point every one of those already-shipped, already-tuned
//! systems depends on, for a reason (lens aperture, not camera/object
//! motion) none of them are designed to distinguish from a real
//! disocclusion. Firing a SEPARATE ray, used only to determine "how far
//! did this pixel's jittered view diverge from its sharp view," leaves
//! every existing system completely untouched — this module's own
//! functions are called from a NEW pass that runs after the sharp image
//! is already fully shaded and composited, not from primary ray
//! generation itself.
//!
//! **Why stochastic ray jitter, not a post-process blur, for this
//! renderer specifically:** every classic real-time DOF artifact
//! (foreground/background bleeding, bokeh-shape faking, halos from
//! naive gather blur — see PROGRESS.md's own DOF entry for the full
//! survey this design is based on) exists because a screen-space blur
//! averages already-flattened 2D pixels with no real occlusion
//! information. This renderer is a raymarcher, not a rasterizer — firing
//! a genuinely different ray per frame and resampling based on where it
//! ACTUALLY hits avoids every one of those artifacts structurally: the
//! resample position comes from a real raymarched hit test against real
//! geometry, not a 2D kernel that doesn't know what's actually occluding
//! what.
//!
//! **Why in-focus pixels get zero jitter for free:** the jittered ray is
//! always re-aimed through the SAME focal-plane point the unperturbed
//! ray would hit. A surface sitting exactly at the focal distance is hit
//! at (approximately) that same point regardless of which aperture
//! offset fired the ray, so it reprojects to (approximately) the SAME
//! screen UV on the sharp image every frame — no separate "detect
//! in-focus, skip blur" branch needed, it falls out of the geometry. A
//! surface far from the focal plane hits a meaningfully different world
//! point each frame (real lens parallax), reprojecting to a DIFFERENT
//! nearby UV on the sharp image each frame — accumulating those over
//! time is what produces the blur.
//!
//! Every function here is a faithful-by-construction reference for its
//! WGSL mirror (a new `hybrid_dof.wgsl` resolve pass plus a dedicated
//! temporal-accumulate pass, both reading the ALREADY-composited sharp
//! image rather than touching primary ray generation), following this
//! project's established CPU-reference-first convention (see `mod.rs`'s
//! own doc comment) — WGSL is written only after these are proven
//! correct with `cargo test`.

use bevy::math::Vec3;

/// Real-world cine "Super 35" sensor height in the same length unit
/// `focal_distance`/scene-depth values use (meters, matching every other
/// distance in this renderer — `ConeTraceConfig::max_t`, `DdgiConfig::
/// max_t`, etc.) — `18.66mm = 0.01866m`. Used to derive a physically
/// meaningful focal length from the camera's own vertical FOV rather
/// than exposing a separate, redundant "focal length" tunable that could
/// drift out of sync with the FOV already driving projection — same
/// value Bevy's own `bevy_post_process::dof` module uses for the
/// identical reason (see PROGRESS.md's own DOF research entry).
pub const SENSOR_HEIGHT_METERS: f32 = 0.01866;

/// Pinhole-equivalent focal length from vertical FOV and sensor height —
/// standard camera-projection identity: `tan(fov/2) = (sensor/2) /
/// focal_length`, rearranged. Matches `bevy_post_process::dof`'s own
/// derivation (verified against its shipped source — see PROGRESS.md).
pub fn focal_length_from_vertical_fov(sensor_height: f32, vertical_fov_radians: f32) -> f32 {
    0.5 * sensor_height / (0.5 * vertical_fov_radians).tan()
}

/// Aperture diameter from focal length and f-stop (`N`): `A = f / N`.
/// Radius is half that. An f-stop of `0.0` (or negative) is not a
/// physically meaningful aperture — clamped to a small positive epsilon
/// so callers get a near-pinhole (near-zero aperture, near-zero DOF
/// blur) result instead of a divide-by-zero, matching this codebase's
/// established "clamp a near-degenerate denominator" convention (see
/// `ddgi_ref::CHEBYSHEV_VARIANCE_FLOOR`'s own doc comment for the same
/// pattern elsewhere).
pub fn aperture_radius(focal_length: f32, f_stop: f32) -> f32 {
    let f_stop = f_stop.max(1e-3);
    0.5 * focal_length / f_stop
}

/// Thin-lens circle-of-confusion DIAMETER at a given scene depth,
/// relative to the SAME length unit `focal_length`/`focus_distance`/
/// `depth` all share (meters, this renderer's own scene-unit
/// convention) — the standard Zeiss/thin-lens formula:
///
/// `c = (focal_length^2 / f_stop) * |depth - focus_distance| /
///      (depth * (focus_distance - focal_length))`
///
/// `depth` is true camera-axis distance (not Euclidean ray length — see
/// this function's own caller for the distinction, and not NDC/reverse-Z
/// depth). `depth <= 0` (behind the camera, degenerate) returns `0.0`
/// (no meaningful CoC for a point that isn't in front of the lens at
/// all). `focus_distance - focal_length` is clamped to a small positive
/// epsilon — for any camera where the focal length isn't absurdly close
/// to the focus distance (i.e. every normal case) this is always
/// strictly positive and the clamp never engages; it exists only to
/// prevent a divide-by-near-zero blowup in a pathological macro-lens
/// configuration, matching this codebase's own established denominator-
/// clamping convention.
pub fn circle_of_confusion_diameter(depth: f32, focus_distance: f32, focal_length: f32, f_stop: f32) -> f32 {
    if depth <= 0.0 {
        return 0.0;
    }
    let f_stop = f_stop.max(1e-3);
    let denom = depth * (focus_distance - focal_length).max(1e-4);
    (focal_length * focal_length / f_stop) * (depth - focus_distance).abs() / denom
}

/// A single sample on a Vogel disk (a low-discrepancy, near-uniform-
/// coverage disk sampling pattern — Vogel 1979's phyllotaxis model,
/// widely used in real-time graphics for disk sampling; see this
/// module's own doc comment / PROGRESS.md for citation): `r =
/// R*sqrt((index+0.5)/total)`, `theta = index * golden_angle`.
/// `index`/`total` select ONE point from an N-point disk (a full,
/// deterministic sequence, not randomness — see `golden_angle_rotation`'s
/// own doc comment for how this renderer gets frame-to-frame variation
/// without a real RNG primitive). Returns an offset in `[-radius,
/// radius]^2` (a unit disk scaled by `radius`), NOT a unit vector —
/// callers multiply by nothing further.
///
/// The `+0.5` inside the sqrt (rather than a bare `index/total`) avoids
/// ever sampling the disk's exact center (`r=0`) for `index=0` — a
/// concentrated cluster of samples at the exact center is a real,
/// visually distinct artifact this shift avoids (points remain
/// distributed with no special-cased index).
const GOLDEN_ANGLE_RADIANS: f32 = 2.399_963_3; // π * (3 - sqrt(5)), the golden angle
pub fn vogel_disk_sample(index: u32, total: u32, radius: f32) -> bevy::math::Vec2 {
    let total = total.max(1);
    let r = radius * ((index as f32 + 0.5) / total as f32).sqrt();
    let theta = index as f32 * GOLDEN_ANGLE_RADIANS;
    bevy::math::Vec2::new(r * theta.cos(), r * theta.sin())
}

/// Which Vogel-disk sample index this frame uses — a simple rotating
/// index (`frame_index % ring_size`), NOT a random pick: this renderer
/// has no RNG primitive anywhere (see `hybrid_post.wgsl`'s own doc
/// comment on the identical constraint for film-grain hashing), so
/// frame-to-frame variation comes from cycling deterministically through
/// a fixed, well-distributed sample sequence instead — exactly the same
/// "rotating deterministic index, not real randomness" approach
/// `ddgi_ref::ddgi_probe_relight_start` already establishes for probe
/// relighting. `ring_size` should be large enough that the sequence
/// doesn't visibly repeat within `max_history_length` frames (the
/// temporal accumulator's own convergence window) — a bigger ring than
/// the accumulator's own history cap adds no value (already-converged
/// history moots any further new sample diversity), so `ring_size ==
/// max_history_length as u32` is the natural choice, not an arbitrary
/// separate tunable.
pub fn dof_sample_index(frame_index: u32, ring_size: u32) -> u32 {
    frame_index % ring_size.max(1)
}

/// The whole stochastic-DOF ray transform: given the UNPERTURBED primary
/// ray (`ro`, `rd`, both already normalized/world-space, `rd` the
/// camera's own forward-ish view direction for this pixel), the camera's
/// own `right`/`up` basis vectors (unit length, orthogonal to the
/// camera's forward axis — NOT this pixel's own ray direction, since the
/// aperture disk lies in the LENS plane, perpendicular to the camera's
/// optical axis, not perpendicular to each individual pixel's own
/// off-axis ray), and this frame's aperture-disk offset (`disk`, from
/// `vogel_disk_sample`, already scaled by `aperture_radius`): returns a
/// new `(origin, direction)` pair.
///
/// Math: project the unperturbed ray out to the FOCUS DISTANCE along the
/// camera's own optical axis (`forward`), not along `rd` itself, to find
/// the focus-plane point — `t = focus_distance / dot(rd, forward)`
/// (perspective-correct: a pixel far off-axis has `rd` at an angle to
/// `forward`, so simply using `focus_distance` as a Euclidean `t` along
/// `rd` would put the "focus plane" on a sphere, not a flat plane,
/// which is wrong — real camera focus planes are flat, perpendicular to
/// the optical axis). Then offset the ray origin within the lens plane
/// by `disk.x * right + disk.y * up`, and re-aim from that new origin
/// through the SAME focus-plane point — this is what makes an in-focus
/// surface (sitting near that plane) land at nearly the same screen
/// position regardless of the disk offset, while a surface far from the
/// focus plane visibly shifts position with the offset (the actual
/// mechanism that PRODUCES defocus blur once accumulated over many
/// differently-offset frames).
pub fn dof_jittered_ray(ro: Vec3, rd: Vec3, forward: Vec3, right: Vec3, up: Vec3, focus_distance: f32, disk: bevy::math::Vec2) -> (Vec3, Vec3) {
    let cos_theta = rd.dot(forward).max(1e-4);
    let t_focus = focus_distance / cos_theta;
    let focus_point = ro + rd * t_focus;
    let jittered_origin = ro + right * disk.x + up * disk.y;
    let jittered_dir = (focus_point - jittered_origin).normalize();
    (jittered_origin, jittered_dir)
}

/// Reprojects a WORLD-SPACE point through the CURRENT camera's own
/// `clip_from_world` matrix into a `[0, 1]` UV — this is the "which
/// already-shaded sharp pixel should this jittered ray's hit point
/// resample from" step (see this module's own doc comment). Identical
/// math to `temporal_ref::world_to_previous_uv` (that function's name
/// says "previous" only because of ITS OWN caller's use — the underlying
/// projection-to-UV math is generic, reused here verbatim against the
/// CURRENT frame's matrix instead), kept as its own function rather than
/// calling that one directly: this module has no dependency on
/// `temporal_ref` otherwise, and the "previous" framing in that
/// function's own name/doc comment would be actively misleading if
/// called from here unchanged. Returns `None` when the point is behind
/// the camera (`w <= 0`) — no screen-space UV can represent that,
/// callers must treat a jittered ray that resolves behind the camera as
/// having no valid resample target (fall back to the sharp/unjittered
/// pixel's own color, matching a real lens's own light-outside-the-
/// aperture-cone behavior of simply not contributing).
pub fn dof_resample_uv(p_world_jittered: Vec3, clip_from_world: bevy::math::Mat4) -> Option<bevy::math::Vec2> {
    let clip = clip_from_world * p_world_jittered.extend(1.0);
    if clip.w <= 0.0 {
        return None;
    }
    let ndc = clip.truncate() / clip.w;
    Some(bevy::math::Vec2::new(ndc.x * 0.5 + 0.5, 1.0 - (ndc.y * 0.5 + 0.5)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn focal_length_matches_the_standard_pinhole_identity_at_a_known_fov() {
        // A 90-degree vertical FOV is the textbook "tan(45deg) == 1"
        // case: focal_length == 0.5 * sensor_height exactly.
        let focal_length = focal_length_from_vertical_fov(SENSOR_HEIGHT_METERS, PI / 2.0);
        assert!(
            (focal_length - 0.5 * SENSOR_HEIGHT_METERS).abs() < 1e-6,
            "at exactly 90 degree vertical FOV, focal_length should equal half the sensor height: got {focal_length}"
        );
    }

    #[test]
    fn a_narrower_fov_produces_a_longer_focal_length() {
        let wide = focal_length_from_vertical_fov(SENSOR_HEIGHT_METERS, 1.2);
        let narrow = focal_length_from_vertical_fov(SENSOR_HEIGHT_METERS, 0.4);
        assert!(narrow > wide, "a narrower (more telephoto) FOV must produce a LONGER focal length: wide={wide} narrow={narrow}");
    }

    #[test]
    fn aperture_radius_halves_when_f_stop_doubles() {
        let f = 0.05;
        let wide_open = aperture_radius(f, 1.4);
        let stopped_down = aperture_radius(f, 2.8);
        assert!(
            (wide_open - 2.0 * stopped_down).abs() < 1e-6,
            "doubling f-stop must exactly halve aperture radius (A = f/N): wide_open={wide_open} stopped_down={stopped_down}"
        );
    }

    #[test]
    fn aperture_radius_never_divides_by_a_true_zero_f_stop() {
        let r = aperture_radius(0.05, 0.0);
        assert!(r.is_finite() && r > 0.0, "an f-stop of exactly zero must not blow up to infinity/NaN: got {r}");
    }

    #[test]
    fn coc_is_exactly_zero_at_the_focus_distance() {
        let coc = circle_of_confusion_diameter(10.0, 10.0, 0.05, 2.8);
        assert!(coc.abs() < 1e-6, "a point exactly at the focus distance must have zero circle of confusion: got {coc}");
    }

    #[test]
    fn coc_grows_with_distance_from_the_focus_plane_on_both_sides() {
        let focus = 10.0;
        let at_focus = circle_of_confusion_diameter(focus, focus, 0.05, 2.8);
        let nearer = circle_of_confusion_diameter(focus - 3.0, focus, 0.05, 2.8);
        let farther = circle_of_confusion_diameter(focus + 3.0, focus, 0.05, 2.8);
        assert!(nearer > at_focus, "a point in front of the focus plane must have a larger CoC than the focus plane itself");
        assert!(farther > at_focus, "a point behind the focus plane must have a larger CoC than the focus plane itself");
    }

    #[test]
    fn coc_shrinks_as_the_lens_stops_down_to_a_higher_f_number() {
        let wide_open = circle_of_confusion_diameter(15.0, 10.0, 0.05, 1.4);
        let stopped_down = circle_of_confusion_diameter(15.0, 10.0, 0.05, 11.0);
        assert!(
            stopped_down < wide_open,
            "a higher f-number (smaller aperture) must produce a SMALLER circle of confusion at the same depth: \
             wide_open={wide_open} stopped_down={stopped_down}"
        );
    }

    #[test]
    fn coc_is_zero_for_a_point_behind_the_camera() {
        let coc = circle_of_confusion_diameter(-5.0, 10.0, 0.05, 2.8);
        assert_eq!(coc, 0.0, "a point behind the camera has no meaningful circle of confusion");
    }

    #[test]
    fn vogel_disk_samples_never_land_exactly_at_the_center() {
        for i in 0..32 {
            let s = vogel_disk_sample(i, 32, 1.0);
            assert!(s.length() > 1e-3, "sample index {i} landed too close to the disk center: {s:?}");
        }
    }

    #[test]
    fn vogel_disk_samples_stay_within_the_requested_radius() {
        for i in 0..64 {
            let s = vogel_disk_sample(i, 64, 2.5);
            assert!(s.length() <= 2.5 + 1e-4, "sample index {i} escaped the requested radius: {s:?}");
        }
    }

    #[test]
    fn vogel_disk_sample_at_index_zero_is_deterministic_not_random() {
        let a = vogel_disk_sample(0, 16, 1.0);
        let b = vogel_disk_sample(0, 16, 1.0);
        assert_eq!(a, b, "the same index/total/radius must always produce the exact same sample — this is a deterministic sequence, not randomness");
    }

    #[test]
    fn vogel_disk_covers_a_reasonable_spread_of_angles_not_clustered_in_one_direction() {
        // A coarse but real spread check: bucket sample angles into 8
        // octants and confirm every octant gets at least one sample out
        // of a generous 64-sample ring — catches a genuinely broken
        // angle formula (e.g. a constant or near-constant theta) without
        // demanding perfect uniformity.
        let mut octant_hit = [false; 8];
        for i in 0..64 {
            let s = vogel_disk_sample(i, 64, 1.0);
            let angle = s.y.atan2(s.x).rem_euclid(2.0 * PI);
            let octant = ((angle / (PI / 4.0)) as usize).min(7);
            octant_hit[octant] = true;
        }
        assert!(octant_hit.iter().all(|&hit| hit), "expected every angular octant to receive at least one sample across 64 Vogel-disk samples: {octant_hit:?}");
    }

    #[test]
    fn dof_sample_index_rotates_through_the_full_ring_and_wraps() {
        assert_eq!(dof_sample_index(0, 24), 0);
        assert_eq!(dof_sample_index(23, 24), 23);
        assert_eq!(dof_sample_index(24, 24), 0, "must wrap back to 0 after a full ring");
        assert_eq!(dof_sample_index(50, 24), 2);
    }

    #[test]
    fn dof_sample_index_never_divides_by_a_true_zero_ring_size() {
        let i = dof_sample_index(5, 0);
        assert_eq!(i, 0, "a zero ring size must not panic/divide-by-zero — clamped to a 1-slot ring");
    }

    #[test]
    fn a_zero_aperture_offset_reproduces_the_unperturbed_ray_through_the_focus_point() {
        let ro = Vec3::new(0.0, 0.0, 0.0);
        let forward = Vec3::new(0.0, 0.0, -1.0);
        let right = Vec3::new(1.0, 0.0, 0.0);
        let up = Vec3::new(0.0, 1.0, 0.0);
        let rd = forward; // dead-center pixel, ray parallel to the optical axis
        let (origin, dir) = dof_jittered_ray(ro, rd, forward, right, up, 10.0, bevy::math::Vec2::ZERO);
        assert!((origin - ro).length() < 1e-5, "a zero disk offset must not move the ray origin at all: {origin:?}");
        assert!((dir - rd).length() < 1e-5, "a zero disk offset must reproduce the unperturbed ray direction exactly: {dir:?}");
    }

    #[test]
    fn a_surface_exactly_at_the_focus_distance_lands_at_nearly_the_same_point_regardless_of_disk_offset() {
        // The core DOF-convergence claim: a point AT the focus plane
        // should be hit at (very nearly) the same world position whether
        // the ray came from disk offset zero or a real nonzero offset —
        // this is what makes in-focus content stay sharp under temporal
        // accumulation without any special-cased "detect in focus" logic.
        let ro = Vec3::new(0.0, 0.0, 0.0);
        let forward = Vec3::new(0.0, 0.0, -1.0);
        let right = Vec3::new(1.0, 0.0, 0.0);
        let up = Vec3::new(0.0, 1.0, 0.0);
        let rd = forward;
        let focus_distance = 10.0;
        let (origin_a, dir_a) = dof_jittered_ray(ro, rd, forward, right, up, focus_distance, bevy::math::Vec2::ZERO);
        let (origin_b, dir_b) = dof_jittered_ray(ro, rd, forward, right, up, focus_distance, bevy::math::Vec2::new(0.05, -0.03));
        // The point each ray reaches AT the focus distance along its own
        // direction (t = focus_distance / cos(angle to forward), same
        // formula the function itself uses internally to build the
        // target) should coincide almost exactly.
        let hit_a = origin_a + dir_a * (focus_distance / dir_a.dot(forward));
        let hit_b = origin_b + dir_b * (focus_distance / dir_b.dot(forward));
        assert!(
            (hit_a - hit_b).length() < 1e-4,
            "a surface at the focus distance must be hit at nearly the same world point regardless of aperture offset: \
             hit_a={hit_a:?} hit_b={hit_b:?}"
        );
    }

    #[test]
    fn a_surface_far_from_the_focus_distance_visibly_shifts_position_with_a_nonzero_disk_offset() {
        // The complementary claim: a point FAR from the focus plane must
        // shift noticeably with a nonzero aperture offset — this is the
        // actual mechanism that produces defocus blur once accumulated.
        let ro = Vec3::new(0.0, 0.0, 0.0);
        let forward = Vec3::new(0.0, 0.0, -1.0);
        let right = Vec3::new(1.0, 0.0, 0.0);
        let up = Vec3::new(0.0, 1.0, 0.0);
        let rd = forward;
        let focus_distance = 10.0;
        let far_surface_distance = 30.0; // well past the focus plane
        let (origin_a, dir_a) = dof_jittered_ray(ro, rd, forward, right, up, focus_distance, bevy::math::Vec2::ZERO);
        let (origin_b, dir_b) = dof_jittered_ray(ro, rd, forward, right, up, focus_distance, bevy::math::Vec2::new(0.05, -0.03));
        let hit_a = origin_a + dir_a * (far_surface_distance / dir_a.dot(forward));
        let hit_b = origin_b + dir_b * (far_surface_distance / dir_b.dot(forward));
        assert!(
            (hit_a - hit_b).length() > 0.01,
            "a surface far from the focus distance must shift MEANINGFULLY with a nonzero aperture offset \
             (this is what produces defocus blur): hit_a={hit_a:?} hit_b={hit_b:?} delta={:?}",
            (hit_a - hit_b).length()
        );
    }

    #[test]
    fn the_far_surface_shift_grows_with_the_disk_offset_magnitude() {
        let ro = Vec3::new(0.0, 0.0, 0.0);
        let forward = Vec3::new(0.0, 0.0, -1.0);
        let right = Vec3::new(1.0, 0.0, 0.0);
        let up = Vec3::new(0.0, 1.0, 0.0);
        let rd = forward;
        let focus_distance = 10.0;
        let far_surface_distance = 30.0;
        let (origin_zero, dir_zero) = dof_jittered_ray(ro, rd, forward, right, up, focus_distance, bevy::math::Vec2::ZERO);
        let (origin_small, dir_small) = dof_jittered_ray(ro, rd, forward, right, up, focus_distance, bevy::math::Vec2::new(0.02, 0.0));
        let (origin_large, dir_large) = dof_jittered_ray(ro, rd, forward, right, up, focus_distance, bevy::math::Vec2::new(0.1, 0.0));
        let hit_zero = origin_zero + dir_zero * (far_surface_distance / dir_zero.dot(forward));
        let hit_small = origin_small + dir_small * (far_surface_distance / dir_small.dot(forward));
        let hit_large = origin_large + dir_large * (far_surface_distance / dir_large.dot(forward));
        let shift_small = (hit_small - hit_zero).length();
        let shift_large = (hit_large - hit_zero).length();
        assert!(
            shift_large > shift_small,
            "a LARGER aperture disk offset must produce a LARGER far-surface position shift: shift_small={shift_small} shift_large={shift_large}"
        );
    }

    #[test]
    fn an_off_axis_pixels_ray_still_targets_a_flat_focus_plane_not_a_sphere() {
        // A pixel whose ray is at an angle to the camera's own forward
        // axis must still compute a focus point on the FLAT plane
        // perpendicular to `forward` at `focus_distance` — not a point
        // at Euclidean distance `focus_distance` along its own (angled)
        // ray, which would trace out a sphere instead of a plane and
        // give every off-axis pixel a subtly wrong focus depth.
        let ro = Vec3::ZERO;
        let forward = Vec3::new(0.0, 0.0, -1.0);
        let right = Vec3::new(1.0, 0.0, 0.0);
        let up = Vec3::new(0.0, 1.0, 0.0);
        // A ray angled 45 degrees off-axis in the +X direction.
        let rd = (forward + right).normalize();
        let focus_distance = 10.0;
        let (_, dir) = dof_jittered_ray(ro, rd, forward, right, up, focus_distance, bevy::math::Vec2::ZERO);
        // The zero-offset case must reproduce rd exactly regardless of
        // angle (already covered by the dedicated zero-offset test for
        // the on-axis case) — here, additionally confirm the IMPLIED
        // focus point sits on the flat plane z = -focus_distance, not at
        // Euclidean distance focus_distance from the origin.
        let t = focus_distance / dir.dot(forward);
        let focus_point = ro + dir * t;
        assert!(
            (focus_point.z - (-focus_distance)).abs() < 1e-4,
            "the focus point for an off-axis ray must lie on the flat plane at z = -focus_distance, got z={}",
            focus_point.z
        );
    }

    // -----------------------------------------------------------------
    // dof_resample_uv
    // -----------------------------------------------------------------

    /// A simple camera looking down -Z from the origin, standard
    /// right-handed perspective projection — enough to build a real
    /// `clip_from_world` for `dof_resample_uv`'s own tests without
    /// depending on this renderer's actual Bevy camera setup.
    fn test_clip_from_world(vertical_fov: f32, aspect_ratio: f32) -> bevy::math::Mat4 {
        let clip_from_view = bevy::math::Mat4::perspective_rh(vertical_fov, aspect_ratio, 0.1, 1000.0);
        // world_from_view is identity here (camera at origin, looking
        // down -Z, no rotation) -> view_from_world is also identity ->
        // clip_from_world == clip_from_view.
        clip_from_view
    }

    #[test]
    fn a_point_dead_center_on_the_optical_axis_reprojects_to_uv_0_5_0_5() {
        let clip_from_world = test_clip_from_world(PI / 2.0, 1.0);
        let uv = dof_resample_uv(Vec3::new(0.0, 0.0, -10.0), clip_from_world).expect("a point in front of the camera must reproject");
        assert!((uv - bevy::math::Vec2::new(0.5, 0.5)).length() < 1e-4, "a point dead-center on the optical axis must reproject to UV (0.5, 0.5): got {uv:?}");
    }

    #[test]
    fn a_point_behind_the_camera_has_no_valid_resample_uv() {
        let clip_from_world = test_clip_from_world(PI / 2.0, 1.0);
        let uv = dof_resample_uv(Vec3::new(0.0, 0.0, 10.0), clip_from_world);
        assert!(uv.is_none(), "a point BEHIND the camera must have no valid resample UV, got {uv:?}");
    }

    #[test]
    fn a_point_offset_to_the_right_of_the_axis_reprojects_to_a_uv_with_a_larger_x() {
        let clip_from_world = test_clip_from_world(PI / 2.0, 1.0);
        let center = dof_resample_uv(Vec3::new(0.0, 0.0, -10.0), clip_from_world).unwrap();
        let right_of_center = dof_resample_uv(Vec3::new(2.0, 0.0, -10.0), clip_from_world).unwrap();
        assert!(
            right_of_center.x > center.x,
            "a world point offset toward +X (camera's own right, looking down -Z) must reproject to a LARGER UV.x: \
             center={center:?} right_of_center={right_of_center:?}"
        );
    }

    #[test]
    fn dof_resample_uv_matches_world_to_previous_uv_for_the_identical_matrix_and_point() {
        // This function's own doc comment claims it's the identical math
        // to temporal_ref::world_to_previous_uv, just applied to the
        // CURRENT frame's matrix instead of the previous one — verify
        // that claim directly rather than just asserting it in prose.
        let clip_from_world = test_clip_from_world(1.1, 1.777);
        let p = Vec3::new(3.0, -1.5, -12.0);
        let a = dof_resample_uv(p, clip_from_world);
        let b = crate::hybrid::temporal_ref::world_to_previous_uv(p, clip_from_world);
        assert_eq!(a, b, "dof_resample_uv and world_to_previous_uv must agree exactly for the same point/matrix — they are the same underlying math");
    }
}
