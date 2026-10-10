//! Collision and occlusion: from the rig's desired pose to one that never
//! shows the inside of a wall.
//!
//! Run once per frame on the *blended* pose. The steps, in order, are those
//! of `docs/knowledge/gameplay-camera/camera-collision-and-occlusion-techniques.md`:
//!
//! 1. **A free start.** Sweeps begin at the pivot, inside the character. If
//!    the pivot itself is embedded (head pressed under a ledge), the next of
//!    `safe_heights` above the target's root that is free is used instead.
//! 2. **Shoulder.** Sweep start → shoulder point, so a shoulder offset into
//!    a wall slides in rather than tunnelling. When the boom from this side
//!    has less than half its length and the other side gives clearly more,
//!    the camera eases over to the other shoulder until its own side has
//!    room again: a shoulder hugging a wall can leave the boom no room at
//!    all.
//! 3. **Boom.** Sweep the shoulder → desired eye with a sphere at least the
//!    near plane's half-diagonal. Then decide:
//!    - the camera's position at its current boom is inside geometry, or
//!      its own move since last frame passed through some (a swing across
//!      a wall): **collision**, snap in now;
//!    - otherwise something came between a camera in free space and the
//!      character: **occlusion**, pull in only after `min_occlusion_time`
//!      (a passing pole does not pump the boom);
//!    - the way out is clear: **hold** `hold_time`, then ease out on a
//!      half-life, never past what the sweep allows.
//! 4. **Feelers.** Side sweeps (Lyra's table), re-traced round-robin one per
//!    frame, cap the distance softly and early: the camera starts closing
//!    before the main sweep is blocked.
//! 5. **Ceiling.** A sweep up from the shoulder gives the steepest pitch at
//!    which the full boom still fits under the ceiling; the orbit eases its
//!    pitch under that next frame instead of the boom crushing in.
//! 6. **Fallback.** A boom stuck short for `fallback_after` seconds blends to
//!    a high view (Gothic's top-down fallback), with hysteresis to leave it.
//! 7. **Fade.** How much to fade the character as the eye nears it.

use super::probe::{CameraProbe, ProbeHit};
use super::rig::{orbit_rotation, yaw_of, DesiredPose};
use crate::math::angle::damp;
use bevy::prelude::*;
use serde::{Deserialize, Serialize};

/// A predictive side sweep: the boom direction turned by `yaw` and `pitch`
/// (degrees), its cap softened by `weight` (1 = as hard as the boom).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Reflect)]
pub struct Feeler {
    pub yaw: f32,
    pub pitch: f32,
    pub weight: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
pub struct CollisionParams {
    /// Smallest probe radius, metres; the near plane's half-diagonal is used
    /// when larger.
    pub min_probe_radius: f32,
    /// Occlusion shorter than this is ignored, seconds.
    pub min_occlusion_time: f32,
    /// After pulling in, hold this long before easing out, seconds.
    pub hold_time: f32,
    pub ease_out_halflife: f32,
    /// How fast a feeler's soft cap pulls the camera in.
    pub feeler_halflife: f32,
    pub feelers: Vec<Feeler>,
    /// Heights above the target's root to start sweeps from when the pivot
    /// is embedded, metres, in order of preference.
    pub safe_heights: Vec<f32>,
    /// A boom shorter than this for `fallback_after` seconds goes high.
    pub fallback_below: f32,
    /// ... and comes back once the free boom is at least this long for
    /// `fallback_after` seconds.
    pub fallback_release: f32,
    pub fallback_after: f32,
    /// The high view: elevation (degrees) and boom (metres).
    pub fallback_pitch: f32,
    pub fallback_distance: f32,
    pub fallback_halflife: f32,
    /// The character fades from fully visible at `fade_far` to fully faded
    /// at `fade_near`, eye-to-pivot metres.
    pub fade_near: f32,
    pub fade_far: f32,
    /// Ease pitch under a low ceiling rather than crushing the boom.
    pub ceiling_cap: bool,
    /// Swap to the other shoulder when the boom from this side has less
    /// than this fraction of its length and the other side gives a quarter
    /// boom more; swap back once this side has 90%. 0 disables.
    pub swap_below: f32,
    pub swap_halflife: f32,
    /// The collision layers that block the camera (an avian `LayerMask`):
    /// by default everything but camera-transparent geometry and ragdolls.
    pub blockers: u32,
}

impl Default for CollisionParams {
    fn default() -> Self {
        // Lyra's feeler table without its main ray (the boom sweep is that).
        let feeler = |yaw, pitch, weight| Feeler { yaw, pitch, weight };
        Self {
            min_probe_radius: 0.15,
            min_occlusion_time: 0.1,
            hold_time: 0.2,
            ease_out_halflife: 0.3,
            feeler_halflife: 0.12,
            feelers: vec![
                feeler(16.0, 0.0, 0.75),
                feeler(-16.0, 0.0, 0.75),
                feeler(32.0, 0.0, 0.5),
                feeler(-32.0, 0.0, 0.5),
                feeler(0.0, 20.0, 1.0),
                feeler(0.0, -20.0, 0.5),
            ],
            safe_heights: vec![1.2, 0.9, 0.6],
            fallback_below: 0.6,
            fallback_release: 1.2,
            fallback_after: 0.3,
            fallback_pitch: 70.0,
            fallback_distance: 1.5,
            fallback_halflife: 0.2,
            fade_near: 0.35,
            fade_far: 0.8,
            ceiling_cap: true,
            swap_below: 0.5,
            swap_halflife: 0.12,
            blockers: crate::physics_avian::layers::CAMERA_BLOCKERS.0,
        }
    }
}

/// The radius that keeps the whole near plane out of geometry: the distance
/// from the eye to a near-plane corner.
pub fn near_plane_radius(near: f32, fov_y: f32, aspect: f32) -> f32 {
    let t = (fov_y * 0.5).tan();
    near * (1.0 + t * t * (1.0 + aspect * aspect)).sqrt()
}

#[derive(Debug, Clone, Default, PartialEq, Reflect)]
pub struct CollisionState {
    /// The boom length in use, from the (slid) shoulder point; `None` until
    /// the first frame.
    pub distance: Option<f32>,
    pub occluded_for: f32,
    /// Seconds since the boom last pulled in.
    pub since_pull_in: f32,
    /// Each feeler's last cap, as a fraction of the boom.
    pub feeler_caps: Vec<f32>,
    pub next_feeler: usize,
    pub short_for: f32,
    pub clear_for: f32,
    pub fallback_active: bool,
    pub fallback: f32,
    /// The steepest pitch (radians) whose full boom fits under the ceiling
    /// found last frame; the orbit eases under it.
    pub pitch_cap: Option<f32>,
    /// The boom's eye last frame (before the fallback blend).
    pub last_eye: Option<Vec3>,
    /// How far over to the other shoulder, 0-1, while this side is blocked.
    pub shoulder_swap: f32,
}

/// The pose after collision.
#[derive(Debug, Clone, Copy, PartialEq, Reflect)]
pub struct ResolvedPose {
    pub eye: Vec3,
    pub rotation: Quat,
    pub fov: f32,
    pub pivot: Vec3,
    /// Where the sweeps started (the pivot, unless it was embedded).
    pub origin: Vec3,
    /// The shoulder point after sliding.
    pub shoulder: Vec3,
    /// Boom length in use and the desired one, metres.
    pub distance: f32,
    pub desired_distance: f32,
    /// How far the boom could reach this frame (the main sweep).
    pub free_distance: f32,
    /// 0 = show the character, 1 = fully faded.
    pub target_fade: f32,
    pub fallback: f32,
    pub probe_radius: f32,
}

impl ResolvedPose {
    /// No collision at all: the desired pose as is.
    pub fn unresolved(desired: &DesiredPose) -> Self {
        Self {
            eye: desired.eye,
            rotation: desired.rotation,
            fov: desired.fov,
            pivot: desired.pivot,
            origin: desired.pivot,
            shoulder: desired.shoulder,
            distance: desired.distance,
            desired_distance: desired.distance,
            free_distance: desired.distance,
            target_fade: 0.0,
            fallback: 0.0,
            probe_radius: 0.0,
        }
    }
}

/// What resolving needs besides the desired pose.
#[derive(Debug, Clone, Copy)]
pub struct CollisionInput {
    pub desired: DesiredPose,
    /// The target's root (feet), for the safe-start chain.
    pub target_root: Vec3,
    pub probe_radius: f32,
    /// The rig cut this frame: snap to the answer, no easing.
    pub cut: bool,
    pub dt: f32,
}

/// How far a sweep may go: to the hit, less a skin so the point it stops at
/// is not left in contact (a sweep from exact contact can read as a hit at
/// distance 0 on some query back-ends).
fn reach(hit: Option<ProbeHit>, max: f32) -> f32 {
    const SKIN: f32 = 0.01;
    hit.map_or(max, |h| (h.distance - SKIN).max(0.0).min(max))
}

impl CollisionState {
    pub fn resolve(
        &mut self,
        input: &CollisionInput,
        params: &CollisionParams,
        probe: &dyn CameraProbe,
    ) -> ResolvedPose {
        let desired = input.desired;
        let r = input.probe_radius.max(params.min_probe_radius);
        let dt = input.dt;

        // 1. A free start.
        let origin = if !probe.overlaps(desired.pivot, r) {
            desired.pivot
        } else {
            params
                .safe_heights
                .iter()
                .map(|h| input.target_root + Vec3::Y * *h)
                .find(|p| !probe.overlaps(*p, r))
                .unwrap_or(desired.pivot)
        };

        // 2. The shoulder: over the other one while this side is blocked
        //    and that side is open, then slid in against whatever is left.
        let flat_right = Quat::from_rotation_y(yaw_of(desired.rotation * Vec3::NEG_Z)) * Vec3::X;
        let lateral = (desired.shoulder - desired.pivot).dot(flat_right);
        let side_reach = |point: Vec3| {
            let to = point - origin;
            match to.try_normalize() {
                Some(direction) => {
                    let length = to.length();
                    (origin + direction * reach(probe.sweep(origin, direction, length, r), length), length)
                }
                None => (origin, 0.0),
            }
        };
        let length = desired.distance.max(0.0);
        let boom = (desired.rotation * Vec3::Z).normalize();
        let boom_room = |from: Vec3| reach(probe.sweep(from, boom, length, r), length);
        if lateral.abs() > 1.0e-3 && length > 0.0 && params.swap_below > 0.0 {
            // What matters is the boom's room from each side, not how far
            // the shoulder itself slid: a boom angled into a wall is pinned
            // even when the shoulder keeps most of its offset.
            let own = boom_room(side_reach(desired.shoulder).0);
            // The other side is only swept when this one is short.
            let other_better = own < params.swap_below * length && {
                let mirrored = desired.shoulder - flat_right * 2.0 * lateral;
                boom_room(side_reach(mirrored).0) > own + 0.25 * length
            };
            let goal = if other_better {
                1.0
            } else if own > 0.9 * length {
                0.0
            } else {
                self.shoulder_swap
            };
            self.shoulder_swap =
                if input.cut { goal } else { damp(self.shoulder_swap, goal, params.swap_halflife, dt) };
        } else {
            self.shoulder_swap = 0.0;
        }
        let wanted_shoulder = desired.shoulder - flat_right * 2.0 * lateral * self.shoulder_swap;
        let (shoulder, _) = side_reach(wanted_shoulder);

        // 3. The boom.
        let free = boom_room(shoulder);

        // 4. Feelers: one re-traced per frame (all on a cut or the first).
        let right = desired.rotation * Vec3::X;
        let feeler_cap = |feeler: &Feeler| {
            let turn = Quat::from_rotation_y(feeler.yaw.to_radians())
                * Quat::from_axis_angle(right, feeler.pitch.to_radians());
            let fraction = if length > 0.0 {
                reach(probe.sweep(shoulder, turn * boom, length, r), length) / length
            } else {
                1.0
            };
            fraction + (1.0 - fraction) * (1.0 - feeler.weight)
        };
        if self.feeler_caps.len() != params.feelers.len() || self.distance.is_none() || input.cut {
            self.feeler_caps = params.feelers.iter().map(feeler_cap).collect();
        } else if !params.feelers.is_empty() {
            let i = self.next_feeler % params.feelers.len();
            self.feeler_caps[i] = feeler_cap(&params.feelers[i]);
            self.next_feeler = i + 1;
        }
        let soft = self.feeler_caps.iter().copied().fold(1.0f32, f32::min) * length;

        // 3 (cont.): collision, occlusion, hold, ease out.
        let mut d = match self.distance {
            Some(d) if !input.cut => d.min(length),
            _ => free.min(soft),
        };
        if free < d - 1.0e-4 {
            // Collision, not occlusion, when the eye would be inside
            // geometry, or when its own move since last frame passed
            // through some: a swing across a wall leaves it in free space
            // on the far side, but it went through to get there.
            let candidate = shoulder + boom * d;
            let inside = probe.overlaps(candidate, r);
            let crossed = self.last_eye.is_some_and(|last| {
                let step = candidate - last;
                step.try_normalize()
                    .is_some_and(|direction| probe.sweep(last, direction, step.length(), r).is_some())
            });
            if inside || crossed {
                d = free;
                self.occluded_for = 0.0;
                self.since_pull_in = 0.0;
            } else {
                self.occluded_for += dt;
                if self.occluded_for >= params.min_occlusion_time {
                    d = free;
                    self.since_pull_in = 0.0;
                }
            }
        } else {
            self.occluded_for = 0.0;
            if soft < d {
                d = damp(d, soft, params.feeler_halflife, dt).min(free);
                self.since_pull_in = 0.0;
            } else {
                let target = free.min(soft);
                if target > d {
                    self.since_pull_in += dt;
                    if self.since_pull_in >= params.hold_time {
                        d = damp(d, target, params.ease_out_halflife, dt).min(target);
                    }
                }
            }
        }
        self.distance = Some(d);
        let mut eye = shoulder + boom * d;
        self.last_eye = Some(eye);
        let mut rotation = desired.rotation;

        // 5. The ceiling: the steepest pitch whose full boom fits.
        self.pitch_cap = if params.ceiling_cap && length > 0.0 {
            probe
                .sweep(shoulder, Vec3::Y, length, r)
                .map(|hit| (hit.distance / length).clamp(-1.0, 1.0).asin())
        } else {
            None
        };

        // 6. Fallback to a high view when stuck short.
        if d < params.fallback_below {
            self.short_for += dt;
        } else {
            self.short_for = 0.0;
        }
        if free >= params.fallback_release {
            self.clear_for += dt;
        } else {
            self.clear_for = 0.0;
        }
        if !self.fallback_active && self.short_for >= params.fallback_after {
            self.fallback_active = true;
        } else if self.fallback_active && self.clear_for >= params.fallback_after {
            self.fallback_active = false;
        }
        let goal = if self.fallback_active { 1.0 } else { 0.0 };
        self.fallback = if input.cut { goal } else { damp(self.fallback, goal, params.fallback_halflife, dt) };
        if self.fallback > 1.0e-3 {
            // The high view at `fallback_pitch`, or nearly straight above if
            // that is blocked too (backed against a wall, the 70° boom
            // still runs into it): whichever reaches further.
            let yaw = yaw_of(desired.rotation * Vec3::NEG_Z);
            let (high, high_eye) = [params.fallback_pitch, 88.0]
                .into_iter()
                .map(|pitch| {
                    let rotation = orbit_rotation(yaw, pitch.to_radians());
                    let boom = rotation * Vec3::Z;
                    let length = reach(probe.sweep(origin, boom, params.fallback_distance, r), params.fallback_distance);
                    (rotation, origin + boom * length, length)
                })
                .fold(None, |best: Option<(Quat, Vec3, f32)>, c| match best {
                    Some(b) if b.2 >= c.2 - 1.0e-3 => Some(b),
                    _ => Some(c),
                })
                .map(|(rotation, eye, _)| (rotation, eye))
                .unwrap();
            eye = eye.lerp(high_eye, self.fallback);
            rotation = rotation.slerp(high, self.fallback);
        }

        // 7. Fade the character as the eye nears it.
        let near = eye.distance(desired.pivot);
        let fade = 1.0 - smoothstep(params.fade_near, params.fade_far, near);

        ResolvedPose {
            eye,
            rotation,
            fov: desired.fov,
            pivot: desired.pivot,
            origin,
            shoulder,
            distance: d,
            desired_distance: length,
            free_distance: free,
            target_fade: fade,
            fallback: self.fallback,
            probe_radius: r,
        }
    }
}

fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0).max(1.0e-6)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::probe::{NoProbe, SdfProbe};
    use crate::camera::rig::{desired_pose, RigShape};

    const DT: f32 = 1.0 / 60.0;

    /// A camera at yaw 0 (looking along −Z, eye behind at +Z), pivot at
    /// 1.6 m, 3 m boom, level, no shoulder.
    fn pose(yaw: f32, pitch: f32) -> DesiredPose {
        let shape = RigShape { distance: 3.0, height: 0.0, fov: 1.0, shoulder: 0.0 };
        desired_pose(Vec3::new(0.0, 1.6, 0.0), yaw, pitch, &shape, 1.0)
    }

    fn input(desired: DesiredPose) -> CollisionInput {
        CollisionInput { desired, target_root: Vec3::ZERO, probe_radius: 0.15, cut: false, dt: DT }
    }

    fn quiet() -> CollisionParams {
        CollisionParams { feelers: Vec::new(), ceiling_cap: false, ..CollisionParams::default() }
    }

    /// A wall across the boom, 1.5 m behind the character, 0.3 m thick.
    fn wall_behind() -> SdfProbe {
        SdfProbe::default().with_box(Vec3::new(0.0, 1.5, 1.65), Vec3::new(10.0, 3.0, 0.3))
    }

    #[test]
    fn with_nothing_around_the_desired_pose_passes_through() {
        let mut state = CollisionState::default();
        let out = state.resolve(&input(pose(0.3, 0.2)), &quiet(), &NoProbe);
        assert!(out.eye.distance(pose(0.3, 0.2).eye) < 1.0e-5);
    }

    #[test]
    fn swinging_into_a_wall_snaps_in_at_once_and_eases_out_only_after_the_hold() {
        let params = quiet();
        let probe = wall_behind();
        let mut state = CollisionState::default();
        // Start facing away from the wall: the boom points to −Z... no: the
        // eye is at +Z, where the wall is. Start turned away instead.
        state.resolve(&input(pose(std::f32::consts::PI, 0.0)), &params, &probe);
        assert!((state.distance.unwrap() - 3.0).abs() < 1.0e-4, "clear at first");
        // Swing round so the eye would sit inside the wall: collision.
        let out = state.resolve(&input(pose(0.0, 0.0)), &params, &probe);
        // The wall's face, less the probe radius, less the 1 cm skin.
        let limit = 1.5 - 0.15 - 0.01;
        assert!((out.distance - limit).abs() < 1.0e-3, "must snap to {limit} in one frame, got {}", out.distance);
        assert!(!probe.overlaps(out.eye, 0.149), "the eye ends outside the wall");
        // Swing back out: held, then eased.
        let away = pose(std::f32::consts::PI, 0.0);
        let mut held_frames = 0;
        let mut last = out.distance;
        for _ in 0..180 {
            let d = state.resolve(&input(away), &params, &probe).distance;
            if (d - last).abs() < 1.0e-6 {
                held_frames += 1;
            }
            assert!(d >= last - 1.0e-6, "easing out must not shrink");
            last = d;
        }
        let hold = (params.hold_time / DT).round() as usize;
        assert!(held_frames + 1 >= hold, "held {held_frames} frames, hold time is {hold}");
        assert!((last - 3.0).abs() < 0.02, "and eases back out to the full boom, {last}");
    }

    #[test]
    fn a_brief_occlusion_is_ignored_and_a_lasting_one_pulls_in() {
        let params = quiet();
        // A pillar between the character and a camera in free space.
        let pillar = SdfProbe::default().with_box(Vec3::new(0.0, 1.6, 1.5), Vec3::new(0.3, 4.0, 0.3));
        let mut state = CollisionState::default();
        state.resolve(&input(pose(0.0, 0.0)), &params, &NoProbe);
        // Three frames (50 ms) of pillar: ignored.
        for _ in 0..3 {
            let out = state.resolve(&input(pose(0.0, 0.0)), &params, &pillar);
            assert!((out.distance - 3.0).abs() < 1.0e-4, "a 50 ms occlusion pulled in to {}", out.distance);
        }
        // It stays: after min_occlusion_time the camera pulls in front of it.
        let mut out = state.resolve(&input(pose(0.0, 0.0)), &params, &pillar);
        for _ in 0..10 {
            out = state.resolve(&input(pose(0.0, 0.0)), &params, &pillar);
        }
        assert!(out.distance < 1.5, "a lasting occlusion must pull in, at {}", out.distance);
    }

    #[test]
    fn penetration_snaps_in_even_inside_the_occlusion_wait() {
        let params = CollisionParams { min_occlusion_time: 10.0, ..quiet() };
        let mut state = CollisionState::default();
        state.resolve(&input(pose(std::f32::consts::PI, 0.0)), &params, &wall_behind());
        let out = state.resolve(&input(pose(0.0, 0.0)), &params, &wall_behind());
        assert!(out.distance < 1.4, "an eye inside a wall cannot wait, at {}", out.distance);
    }

    #[test]
    fn feelers_close_the_boom_before_the_main_sweep_is_blocked() {
        // A wall just off to the side of the boom: the main sweep is clear,
        // a 16° feeler is not.
        let side = SdfProbe::default().with_box(Vec3::new(1.2, 1.6, 2.5), Vec3::new(1.0, 4.0, 3.0));
        let quiet_params = quiet();
        let mut plain = CollisionState::default();
        let main_only = plain.resolve(&input(pose(0.0, 0.0)), &quiet_params, &side);
        assert!((main_only.distance - 3.0).abs() < 1.0e-4, "the main sweep is clear: {}", main_only.distance);
        let mut with_feelers = CollisionState::default();
        let params = CollisionParams { ceiling_cap: false, ..CollisionParams::default() };
        let mut out = with_feelers.resolve(&input(pose(0.0, 0.0)), &params, &side);
        for _ in 0..60 {
            out = with_feelers.resolve(&input(pose(0.0, 0.0)), &params, &side);
        }
        assert!(out.distance < 2.9, "feelers must pull in early, at {}", out.distance);
    }

    #[test]
    fn an_embedded_pivot_starts_from_the_next_free_height() {
        // A low slab whose underside is at 1.5 m: the 1.6 m pivot is inside.
        let slab = SdfProbe::default().with_box(Vec3::new(0.0, 1.75, 0.0), Vec3::new(4.0, 0.5, 4.0));
        let mut state = CollisionState::default();
        let out = state.resolve(&input(pose(0.0, 0.0)), &quiet(), &slab);
        assert!(out.origin.y < 1.4, "the sweep must start below the slab, at {:?}", out.origin);
        assert!(!slab.overlaps(out.origin, 0.15));
        assert!(!slab.overlaps(out.eye, 0.149), "and the eye ends in free space: {:?}", out.eye);
    }

    #[test]
    fn a_shoulder_offset_into_a_wall_slides_in() {
        let wall = SdfProbe::default().with_box(Vec3::new(0.6, 1.5, 0.0), Vec3::new(0.4, 3.0, 10.0));
        let shape = RigShape { distance: 3.0, height: 0.0, fov: 1.0, shoulder: 0.5 };
        let desired = desired_pose(Vec3::new(0.0, 1.6, 0.0), 0.0, 0.0, &shape, 1.0);
        let params = CollisionParams { feelers: Vec::new(), ..CollisionParams::default() };
        let mut state = CollisionState::default();
        let out = state.resolve(&input(desired), &params, &wall);
        assert!(out.shoulder.x < 0.4 - 0.149, "shoulder must stop short of the wall: {:?}", out.shoulder);
        assert!(!wall.overlaps(out.eye, 0.149));
        // The boom runs along the wall the shoulder now touches: neither it
        // nor the ceiling sweep may read that contact as a hit (a live
        // camera once collapsed to 0 m and pitched flat here).
        assert!((out.distance - 3.0).abs() < 1.0e-3, "the boom collapsed to {}", out.distance);
        assert!(state.pitch_cap.is_none(), "no roof, but a pitch cap of {:?}", state.pitch_cap);
    }

    #[test]
    fn a_shoulder_blocked_by_a_wall_swaps_to_the_open_side() {
        // A wall just right of the character; the boom angled a little
        // toward it, as a camera turned along a wall is.
        let wall = SdfProbe::default().with_box(Vec3::new(0.55, 1.5, 0.0), Vec3::new(0.4, 3.0, 20.0));
        let shape = RigShape { distance: 3.0, height: 0.0, fov: 1.0, shoulder: 0.3 };
        let desired = desired_pose(Vec3::new(0.0, 1.6, 0.0), 0.15, 0.0, &shape, 1.0);
        let params = quiet();
        let mut state = CollisionState::default();
        let first = state.resolve(&input(desired), &params, &wall);
        let mut out = first;
        for _ in 0..60 {
            out = state.resolve(&input(desired), &params, &wall);
        }
        assert!(state.shoulder_swap > 0.95, "should be over the other shoulder: {}", state.shoulder_swap);
        assert!(out.shoulder.x < -0.25, "the shoulder point is on the open side: {:?}", out.shoulder);
        assert!(out.distance > first.distance + 1.0, "and the boom has room: {} → {}", first.distance, out.distance);
        // Without the swap the boom is stuck short against the wall.
        let mut stuck = CollisionState::default();
        let no_swap = CollisionParams { swap_below: 0.0, ..quiet() };
        let mut pinned = stuck.resolve(&input(desired), &no_swap, &wall);
        for _ in 0..60 {
            pinned = stuck.resolve(&input(desired), &no_swap, &wall);
        }
        assert!(pinned.distance < out.distance - 1.0, "control: {} vs {}", pinned.distance, out.distance);
        // Clear of the wall, it swaps back.
        for _ in 0..60 {
            state.resolve(&input(desired), &params, &NoProbe);
        }
        assert!(state.shoulder_swap < 0.05, "back over its own shoulder: {}", state.shoulder_swap);
    }

    #[test]
    fn a_low_ceiling_caps_the_pitch() {
        let roof = SdfProbe::default().with_box(Vec3::new(0.0, 2.75, 0.0), Vec3::new(10.0, 0.3, 10.0));
        let params = CollisionParams { feelers: Vec::new(), ..CollisionParams::default() };
        let mut state = CollisionState::default();
        state.resolve(&input(pose(0.0, 0.2)), &params, &roof);
        let cap = state.pitch_cap.expect("a roof 1 m above must cap the pitch");
        // 2.6 m underside − 0.15 radius − 1.6 m shoulder = 0.85 m over a 3 m boom.
        assert!((cap - (0.85f32 / 3.0).asin()).abs() < 2.0e-3, "cap {cap}");
        assert!(CollisionState::default().resolve(&input(pose(0.0, 0.2)), &params, &NoProbe).distance > 0.0);
    }

    #[test]
    fn the_fallback_waits_for_a_sustained_short_boom_and_leaves_with_hysteresis() {
        let params = quiet();
        // A wall 0.5 m behind: the boom is stuck at 0.35 m.
        let tight = SdfProbe::default().with_box(Vec3::new(0.0, 1.5, 0.65), Vec3::new(10.0, 3.0, 0.3));
        let mut state = CollisionState::default();
        for i in 0..12 {
            let out = state.resolve(&input(pose(0.0, 0.0)), &params, &tight);
            if i < 10 {
                assert!(out.fallback < 1.0e-3, "fallback engaged after only {i} frames");
            }
        }
        let mut out = state.resolve(&input(pose(0.0, 0.0)), &params, &tight);
        for _ in 0..60 {
            out = state.resolve(&input(pose(0.0, 0.0)), &params, &tight);
        }
        assert!(out.fallback > 0.95, "a stuck boom must go high, fallback {}", out.fallback);
        assert!(out.eye.y > 2.0, "the high view is above the character: {:?}", out.eye);
        // Free again: stays high for the release time, then comes back.
        let mut free = state.resolve(&input(pose(0.0, 0.0)), &params, &NoProbe);
        assert!(free.fallback > 0.9, "leaving must wait out the release time");
        for _ in 0..120 {
            free = state.resolve(&input(pose(0.0, 0.0)), &params, &NoProbe);
        }
        assert!(free.fallback < 0.05, "and then return, fallback {}", free.fallback);
    }

    #[test]
    fn the_character_fades_as_the_eye_closes_in() {
        let params = quiet();
        let mut state = CollisionState::default();
        let far = state.resolve(&input(pose(0.0, 0.0)), &params, &NoProbe);
        assert_eq!(far.target_fade, 0.0);
        let tight = SdfProbe::default().with_box(Vec3::new(0.0, 1.5, 0.6), Vec3::new(10.0, 3.0, 0.3));
        let near = CollisionState::default().resolve(&input(pose(0.0, 0.0)), &params, &tight);
        assert!(near.target_fade > 0.9, "an eye 0.3 m away must fade the character: {}", near.target_fade);
    }

    #[test]
    fn the_near_plane_radius_reaches_the_corners() {
        // Bevy defaults: near 0.1, 45° vertical FOV, 16:9.
        let r = near_plane_radius(0.1, std::f32::consts::FRAC_PI_4, 16.0 / 9.0);
        assert!((r - 0.131).abs() < 1.0e-3, "r = {r}");
    }
}
