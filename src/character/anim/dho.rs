//! Stage 1 — per-joint damped harmonic oscillators in quaternion space.
//!
//! This is what replaces timeline interpolation. There is no blend window,
//! no easing curve, and no crossfade anywhere in the runtime: each bone is
//! a spring that is continuously pulled toward whatever the current target
//! pose says, and the motion between poses is an *emergent* property of the
//! spring rather than an authored one.
//!
//! # What that buys
//!
//! - **Secondary motion for free.** Slack springs on a heavy limb make it
//!   lag the torso and overshoot on the way back, which is exactly the
//!   follow-through an animator would otherwise hand-key. Tightening the
//!   same springs makes the same pose data read as a snappy jab. The pose
//!   data does not change; only `halflife`/`damping_ratio` do.
//! - **Interruptions stay smooth by construction.** A spring has state, so
//!   redirecting it mid-flight continues from the current position *and
//!   velocity*. There is no moment at which two animations are averaged.
//! - **No authored timing to desynchronise.** The rig is never "3 frames
//!   into a 12-frame blend", so nothing can be interrupted at a bad frame.
//!
//! # Why the dynamics happen in tangent space
//!
//! A quaternion cannot be integrated componentwise — adding a velocity to
//! `(x, y, z, w)` leaves the unit hypersphere immediately. So each bone's
//! spring runs on the **scaled-angle-axis** of its error, a plain `Vec3`
//! that adds and scales like any vector, and the result is mapped back to a
//! quaternion once per step. See [`super::math::quat_ext`].
//!
//! Every difference taken here is neighbourhooded first. Skipping that is
//! the single most common quaternion-animation bug: `q` and `-q` name the
//! same orientation, so half the time an un-guarded spring drives a joint
//! the 359-degree way round.
//!
//! # Fixed substeps
//!
//! The spring is stepped at a fixed internal rate regardless of frame time.
//! The closed-form solver in [`super::math::spring`] is already stable at
//! any `dt`, so this is not about stability — it is about **determinism**:
//! two machines running the same inputs should produce the same pose, and a
//! frame-rate-dependent accumulator would break that. It also keeps a
//! frame-time spike from teleporting a limb.
//!
//! What is *rendered* is not the substep state but that state carried the
//! rest of the way to the frame's real time (see [`DhoState::pose`]). A
//! frame is rarely a whole number of substeps, so the substep state alone
//! lags the frame by a varying 0–8.3 ms while everything else the frame
//! shows (root motion, hip height) is exact — and a near-instant limb then
//! pops by however far it moves in that slack. On a 59.96 Hz display the
//! slack drifts across a substep boundary every ~12.5 s, and around each
//! crossing vsync jitter picks 1, 2 or 3 substeps per frame: a walking
//! foot's frame-to-frame velocity jumped 3.5 m/s there, against 0.68 m/s
//! (its own footfall) at an exact 60 Hz. The projection is a pure function
//! of the state and is never fed back, so determinism is untouched.
//!
//! For the same reason each substep chases the target interpolated to its
//! own moment within the frame, not the frame-end target: holding the
//! target for a whole frame left a 1.27 m/s pop where a footfall met a
//! 3-substep frame. With both, 0.71 m/s at 59.96 Hz and 0.31 at 144 Hz —
//! see `locomotion::tests::a_walking_foot_moves_smoothly_at_any_frame_rate`.

use bevy::math::{Quat, Vec3};

use super::math::quat_ext::{from_scaled_angle_axis, neighborhood, to_scaled_angle_axis};
use super::math::spring::SpringParams;
use super::rig::{BoneSet, LocalPose};
use crate::character::skeleton::Bone;

/// The fixed rate the spring integrates at, in seconds.
///
/// 1/120 s: fine enough that even a very stiff bone (a ~0.02 s half-life)
/// resolves smoothly, cheap enough that a 22-bone rig costs nothing
/// measurable. A frame longer than this runs several substeps.
pub const SUBSTEP_SECONDS: f32 = 1.0 / 120.0;

/// The largest frame time that will be simulated in one call.
///
/// A hitch, a breakpoint, or a window drag can hand us an arbitrarily large
/// `dt`. Simulating all of it would spend the catch-up cost in the very
/// frame that is already late, and would snap every limb to its target —
/// visually a teleport. Clamping means the rig runs slightly slow through a
/// hitch and then recovers, which is far less noticeable.
///
/// The superseded module learned the same lesson: its `step_muscle_sim`
/// carries a `dt.min(1/30)` clamp after a measured first-frame explosion.
pub const MAX_FRAME_SECONDS: f32 = 1.0 / 15.0;

/// Below this remaining error (radians) AND [`REST_SNAP_VELOCITY`], a bone
/// is placed exactly on its target with zero velocity.
///
/// Without it a stiff spring reaches the f32 floor and never leaves: the
/// error it reads back from `goal⁻¹ * current` is quantised to a ulp, so
/// the exact spring solution keeps answering it with a velocity that cannot
/// decay (measured: a 0.03 s arm sat at 7.9e-6 rad/s forever, flipping the
/// last bit of its rotation every frame). Both thresholds are far below
/// anything visible, and the velocity one keeps a spring that is merely
/// passing through its target from being stopped there.
const REST_SNAP_ERROR: f32 = 1.0e-6;
const REST_SNAP_VELOCITY: f32 = 1.0e-4;

/// Per-bone spring state for one character.
///
/// Flat `[T; 22]` arrays rather than a map: see [`super::rig`] for why, and
/// note that it also makes the whole solve a pure function over contiguous
/// memory, callable from a test in microseconds with no `World`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DhoState {
    /// Where each bone is as of the last whole substep — the simulation
    /// state. What is rendered is [`Self::pose`], this carried on to the
    /// frame's real time.
    pub current: BoneSet<Quat>,
    /// Each bone's angular velocity, scaled-angle-axis, radians per second.
    pub velocity: BoneSet<Vec3>,
    /// Leftover time not yet consumed by a whole substep.
    accumulator: f32,
    /// `current` stepped on by `accumulator` toward the last target: where
    /// each bone is at the frame's real time. Derived, never fed back.
    presented: BoneSet<Quat>,
    /// The target the previous frame was given, so substeps can see the
    /// target move through the frame instead of jumping at its start.
    previous_target: BoneSet<Quat>,
}

impl DhoState {
    /// A rig sitting exactly at its rest pose, at rest.
    pub const AT_REST: Self = Self {
        current: BoneSet::splat(Quat::IDENTITY),
        velocity: BoneSet::splat(Vec3::ZERO),
        accumulator: 0.0,
        presented: BoneSet::splat(Quat::IDENTITY),
        previous_target: BoneSet::splat(Quat::IDENTITY),
    };

    /// A rig already settled on `pose`, with no residual motion.
    ///
    /// Use this when spawning a character that should *begin* in a pose,
    /// rather than springing into it from rest on the first frame.
    pub fn settled_on(pose: &LocalPose) -> Self {
        Self {
            current: pose.rotations,
            velocity: BoneSet::splat(Vec3::ZERO),
            accumulator: 0.0,
            presented: pose.rotations,
            previous_target: pose.rotations,
        }
    }

    /// The pose to render: the rig at the frame's real time.
    ///
    /// Not `current`, which trails the frame by the unconsumed part of a
    /// substep; see the module note on fixed substeps for what rendering
    /// that cost.
    pub fn pose(&self, root_translation: Vec3) -> LocalPose {
        LocalPose { rotations: self.presented, root_translation }
    }

    /// Whether every bone has effectively stopped moving.
    ///
    /// The exponential solution never reaches its target exactly, so
    /// "settled" needs a tolerance rather than an equality test.
    pub fn is_settled(&self, tolerance_radians_per_second: f32) -> bool {
        self.velocity
            .iter()
            .all(|(_, v)| v.length_squared() <= tolerance_radians_per_second.powi(2))
    }

    /// Advances every bone toward `target` by `dt`.
    ///
    /// Consumes whole [`SUBSTEP_SECONDS`] substeps, carrying the remainder
    /// into the next call, so the result depends only on elapsed time and
    /// not on how that time was divided into frames.
    pub fn advance(
        &mut self,
        target: &LocalPose,
        springs: &BoneSet<SpringParams>,
        dt: f32,
    ) {
        if dt <= 0.0 {
            return;
        }

        let frame = dt.min(MAX_FRAME_SECONDS);
        // The state sits `accumulator` behind the previous frame's time.
        // Each substep chases the target AT ITS OWN MOMENT, interpolated
        // from the previous frame's target to this one, rather than this
        // frame's target throughout: with a whole frame's worth of held
        // target, a near-instant limb met a sharp event (a footfall)
        // differently in a 1-substep frame than in a 3-substep one.
        let mut behind = -self.accumulator;
        self.accumulator += frame;

        while self.accumulator >= SUBSTEP_SECONDS {
            self.accumulator -= SUBSTEP_SECONDS;
            behind += SUBSTEP_SECONDS;
            let along = (behind / frame).clamp(0.0, 1.0);
            let moment = BoneSet::from_fn(|bone| {
                self.previous_target[bone].slerp(target.rotations[bone], along)
            });
            self.substep(&moment, springs, SUBSTEP_SECONDS);
        }
        self.previous_target = target.rotations;

        // Carry a copy the rest of the way to the frame's time, for display.
        self.presented = self.current;
        if self.accumulator > 0.0 {
            for &bone in Bone::ALL.iter() {
                self.presented[bone] = step_bone(
                    self.current[bone],
                    self.velocity[bone],
                    target.rotations[bone],
                    &springs[bone],
                    self.accumulator,
                )
                .0;
            }
        }
    }

    /// One fixed-size integration step.
    fn substep(&mut self, target: &BoneSet<Quat>, springs: &BoneSet<SpringParams>, dt: f32) {
        for &bone in Bone::ALL.iter() {
            (self.current[bone], self.velocity[bone]) = step_bone(
                self.current[bone],
                self.velocity[bone],
                target[bone],
                &springs[bone],
                dt,
            );
        }
    }

    /// Redirects the rig to a new target without losing momentum.
    ///
    /// This is a no-op on the spring state, and that is the point: because a
    /// spring carries its own position and velocity, simply handing
    /// [`Self::advance`] a different target *is* a velocity-continuous
    /// transition. Nothing needs to be blended, faded, or cross-referenced.
    ///
    /// The method exists so call sites can say what they mean, and so this
    /// note has somewhere to live.
    #[inline]
    pub fn retarget(&mut self) {}
}

impl Default for DhoState {
    fn default() -> Self {
        Self::AT_REST
    }
}

/// Steps one bone's spring by `dt`, returning its new rotation and velocity.
fn step_bone(
    current: Quat,
    velocity: Vec3,
    target: Quat,
    params: &SpringParams,
    dt: f32,
) -> (Quat, Vec3) {
    // Put the target in the same hemisphere BEFORE differencing. Omitting
    // this sends the bone the long way round about half the time;
    // `a_bone_springs_the_short_way_round_to_a_negated_target` fails loudly
    // if it is removed.
    let goal = neighborhood(current, target);

    // Work in the tangent space at the target: `error` is how far the bone
    // still has to travel, as a plain rotation vector.
    let error = to_scaled_angle_axis(goal.inverse() * current);

    let (new_error, new_velocity) =
        super::math::spring::spring_vec3(error, velocity, Vec3::ZERO, params, dt);

    if new_error.length() < REST_SNAP_ERROR && new_velocity.length() < REST_SNAP_VELOCITY {
        return (goal, Vec3::ZERO);
    }

    (goal * from_scaled_angle_axis(new_error), new_velocity)
}

/// Per-bone spring tuning for a whole rig.
///
/// Defaults are deliberately uniform and moderately snappy; per-bone and
/// per-group tuning is what the Phase 8 studio exists to make tractable.
pub fn default_springs() -> BoneSet<SpringParams> {
    BoneSet::from_fn(|bone| match bone {
        // The spine carries the most mass and should feel weighty.
        Bone::Spine | Bone::Spine1 | Bone::Spine2 => SpringParams::critical(0.16),
        // The hips are NOT weighty, for the legs' reason below: both legs
        // hang from them, so a lagging pelvis roll swings the feet about
        // the hip joints. A weight shift or the release before a first step
        // rolls the pelvis ~4 degrees over planted feet; at the spine's
        // 0.16 s that lag carried the rendered left toe 4.2 cm off the
        // target's, hidden by the standing foot lock until the first step
        // released it as a slide. The root's translation is not sprung at
        // all, and its rotation has to keep up with it.
        Bone::Hips => SpringParams::critical(0.015),
        // The head settles a little after the neck, which reads as weight.
        Bone::Neck | Bone::Head => SpringParams::critical(0.13),
        // The legs are nearly instant, and not for feel. They carry the
        // gait's CONTACT geometry: root motion moves the body by exactly
        // how far the planted foot travels in the gait's target pose, so a
        // leg spring slow enough to low-pass the stride shortens the swing
        // the rendered foot really makes, and the body out-walks its own
        // planted foot. Measured per stance on the real walk, by leg
        // half-life, at 1 stride/s: 0.12 s (the old shared value) 302 mm of
        // slide, 0.06 s 99, 0.04 s 27, 0.03 s 7.6, 0.02 s 0.7. A low-pass
        // bites harder on a faster stride, and 0.02 s still left 9.4 mm at
        // a brisk 1.6 strides/s; 0.015 keeps both under 5 mm. The gait's
        // own curves are already smooth, so the spring has nothing to add
        // there.
        Bone::LeftUpLeg
        | Bone::LeftLeg
        | Bone::LeftFoot
        | Bone::LeftToeBase
        | Bone::RightUpLeg
        | Bone::RightLeg
        | Bone::RightFoot
        | Bone::RightToeBase => SpringParams::critical(0.015),
        // The arms are quick too, for a related reason: their walking swing
        // is TIMED by the gait (counter to the legs, with an authored lag),
        // and a spring adds its own lag on top — one that grows with the
        // cadence, so nothing can author around it. A critical spring's
        // rate is `ln 2 / halflife`; at the old 0.12 s that is 5.8 rad/s
        // against a ~5.7 rad/s stride, about 88 degrees — a quarter-cycle —
        // and the opposite hand led a forward foot in 50% of samples,
        // chance. 0.03 s keeps a little weight and ~27 degrees of lag,
        // which `gait::ARM_LAG` accounts for.
        Bone::LeftShoulder
        | Bone::LeftArm
        | Bone::LeftForeArm
        | Bone::LeftHand
        | Bone::RightShoulder
        | Bone::RightArm
        | Bone::RightForeArm
        | Bone::RightHand => SpringParams::critical(0.03),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    /// See `quat_ext`'s note: `angle_between` amplifies f32 rounding near
    /// identity, so exactness is measured with `1 - |dot|`.
    fn rotation_mismatch(a: Quat, b: Quat) -> f32 {
        1.0 - a.dot(b).abs()
    }

    fn pose_with(bone: Bone, rotation: Quat) -> LocalPose {
        let mut pose = LocalPose::REST;
        pose.set_rotation(bone, rotation);
        pose
    }

    /// Runs the spring for `seconds` at a realistic frame time.
    fn run(state: &mut DhoState, target: &LocalPose, seconds: f32) {
        let springs = default_springs();
        let dt = 1.0 / 60.0;
        let steps = (seconds / dt).round() as usize;
        for _ in 0..steps {
            state.advance(target, &springs, dt);
        }
    }

    #[test]
    fn a_rig_at_rest_with_a_rest_target_never_moves() {
        // Guards against a settled rig shimmering in the last mantissa bit,
        // which is visible on screen and easy to introduce.
        let mut state = DhoState::AT_REST;
        run(&mut state, &LocalPose::REST, 10.0);

        for &bone in Bone::ALL.iter() {
            assert_eq!(
                state.current[bone],
                Quat::IDENTITY,
                "{} drifted from rest with nothing driving it",
                bone.name(),
            );
            assert_eq!(state.velocity[bone], Vec3::ZERO, "{} gained velocity", bone.name());
        }
    }

    #[test]
    fn a_bone_converges_on_its_target_and_stays_there() {
        let target = pose_with(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, FRAC_PI_2));
        let mut state = DhoState::AT_REST;

        run(&mut state, &target, 2.0);

        assert!(
            rotation_mismatch(state.current[Bone::LeftArm], target.rotation(Bone::LeftArm))
                < 1.0e-5,
            "LeftArm should have reached its target, got {:?}",
            state.current[Bone::LeftArm],
        );
        assert!(
            state.is_settled(0.01),
            "and should be at rest once there, velocities: {:?}",
            state.velocity[Bone::LeftArm],
        );

        // And it must STAY — a spring that keeps nudging is a shimmer.
        // Compared bit for bit: `rotation_mismatch` has a ~6e-8 floor (a
        // unit quaternion's dot with ITSELF need not round to 1), so it
        // cannot tell "held still" from "flipping its last bit" — which is
        // exactly the shimmer a stiff spring used to settle into before
        // `REST_SNAP_ERROR` existed.
        let settled = state.current[Bone::LeftArm];
        run(&mut state, &target, 5.0);
        assert_eq!(
            settled,
            state.current[Bone::LeftArm],
            "a settled bone must not keep moving",
        );
        assert_eq!(state.velocity[Bone::LeftArm], Vec3::ZERO, "nor keep any velocity");
    }

    #[test]
    fn only_the_targeted_bone_moves() {
        let target = pose_with(Bone::RightForeArm, Quat::from_axis_angle(Vec3::Z, 0.8));
        let mut state = DhoState::AT_REST;

        run(&mut state, &target, 1.0);

        for &bone in Bone::ALL.iter() {
            if bone == Bone::RightForeArm {
                continue;
            }
            assert_eq!(
                state.current[bone],
                Quat::IDENTITY,
                "{} moved although only RightForeArm was targeted",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_bone_springs_the_short_way_round_to_a_negated_target() {
        // THE neighbourhooding regression. `q` and `-q` are the same
        // orientation, so a bone already at `q` must not move at all when
        // handed `-q`. Without the guard in `substep` it would swing a full
        // turn. Fails loudly if that guard is ever removed.
        let rotation = Quat::from_axis_angle(Vec3::Y, 1.2);

        let mut state = DhoState::settled_on(&pose_with(Bone::LeftArm, rotation));
        let negated = pose_with(Bone::LeftArm, -rotation);

        // Track the furthest the bone ever strays, not just where it ends
        // up: a long-way-round swing would return to the right place.
        let springs = default_springs();
        let mut furthest = 0.0f32;
        for _ in 0..120 {
            state.advance(&negated, &springs, 1.0 / 60.0);
            furthest = furthest
                .max(rotation_mismatch(state.current[Bone::LeftArm], rotation));
        }

        assert!(
            furthest < 1.0e-5,
            "a bone handed the negation of its own rotation must not move at all, but \
             strayed by {furthest} — neighbourhooding is not being applied",
        );
    }

    #[test]
    fn a_bone_never_takes_more_than_half_a_turn_to_reach_any_target() {
        // The general form of the previous test, swept over a range of
        // targets: no path may exceed pi, because the short way round always
        // exists.
        let springs = default_springs();
        let mut rng = fastrand::Rng::with_seed(0xD40);

        for _ in 0..50 {
            let axis = Vec3::new(
                rng.f32() * 2.0 - 1.0,
                rng.f32() * 2.0 - 1.0,
                rng.f32() * 2.0 - 1.0,
            )
            .normalize_or_zero();
            if axis == Vec3::ZERO {
                continue;
            }
            let target = pose_with(Bone::Head, Quat::from_axis_angle(axis, rng.f32() * 6.0));

            let mut state = DhoState::AT_REST;
            let mut furthest = 0.0f32;
            for _ in 0..180 {
                state.advance(&target, &springs, 1.0 / 60.0);
                furthest =
                    furthest.max(state.current[Bone::Head].angle_between(Quat::IDENTITY));
            }

            assert!(
                furthest <= std::f32::consts::PI + 1.0e-2,
                "a bone travelled {furthest} rad, further than half a turn",
            );
        }
    }

    #[test]
    fn the_same_elapsed_time_gives_the_same_pose_regardless_of_frame_rate() {
        // Determinism, and the reason for the fixed-substep accumulator: a
        // rig must not animate differently on a faster machine.
        let target = pose_with(Bone::Spine, Quat::from_axis_angle(Vec3::X, 0.7));
        let springs = default_springs();

        let mut at_30 = DhoState::AT_REST;
        for _ in 0..30 {
            at_30.advance(&target, &springs, 1.0 / 30.0);
        }

        let mut at_144 = DhoState::AT_REST;
        for _ in 0..144 {
            at_144.advance(&target, &springs, 1.0 / 144.0);
        }

        let mismatch = rotation_mismatch(at_30.current[Bone::Spine], at_144.current[Bone::Spine]);
        assert!(
            mismatch < 1.0e-6,
            "one second of animation must land in the same place at 30fps and 144fps, \
             mismatched by {mismatch}",
        );
    }

    #[test]
    fn stepping_twice_by_half_a_frame_matches_stepping_once() {
        let target = pose_with(Bone::Neck, Quat::from_axis_angle(Vec3::Z, 0.4));
        let springs = default_springs();

        let mut once = DhoState::AT_REST;
        once.advance(&target, &springs, 1.0 / 30.0);

        let mut twice = DhoState::AT_REST;
        twice.advance(&target, &springs, 1.0 / 60.0);
        twice.advance(&target, &springs, 1.0 / 60.0);

        assert!(
            rotation_mismatch(once.current[Bone::Neck], twice.current[Bone::Neck]) < 1.0e-6,
            "the substep accumulator must make frame subdivision irrelevant",
        );
    }

    #[test]
    fn a_huge_frame_time_does_not_teleport_the_rig() {
        // A hitch must not snap every limb to its target. The rig should
        // advance by at most the clamp and then carry on.
        let target = pose_with(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, FRAC_PI_2));
        let springs = default_springs();

        let mut hitched = DhoState::AT_REST;
        hitched.advance(&target, &springs, 10.0);

        let mut clamped = DhoState::AT_REST;
        clamped.advance(&target, &springs, MAX_FRAME_SECONDS);

        assert!(
            rotation_mismatch(hitched.current[Bone::LeftArm], clamped.current[Bone::LeftArm])
                < 1.0e-6,
            "a 10-second frame must be clamped, not simulated in full",
        );
        assert!(
            rotation_mismatch(hitched.current[Bone::LeftArm], target.rotation(Bone::LeftArm))
                > 1.0e-3,
            "and must therefore NOT have arrived at the target already",
        );
    }

    #[test]
    fn retargeting_mid_flight_is_position_and_velocity_continuous() {
        // The headline transition property. A spring redirected mid-flight
        // continues from where it is, at the speed it is going — no blend
        // window, nothing averaged.
        //
        // A property of the spring, not of the default tuning, so it pins
        // its own: redirected after 20 substeps, a very stiff default (the
        // arms are 0.03 s) is 5.5 half-lives settled and nearly stopped,
        // and there is no momentum left worth preserving.
        let springs = BoneSet::from_fn(|_| SpringParams::critical(0.12));
        let dt = 1.0 / 120.0;

        let first = pose_with(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, FRAC_PI_2));
        let second = pose_with(Bone::LeftArm, Quat::from_axis_angle(Vec3::Z, -1.0));

        let mut state = DhoState::AT_REST;
        for _ in 0..20 {
            state.advance(&first, &springs, dt);
        }

        let before = state.current[Bone::LeftArm];
        let velocity_before = state.velocity[Bone::LeftArm];
        assert!(
            velocity_before.length() > 0.5,
            "test setup: the bone should be moving briskly, got {}",
            velocity_before.length(),
        );

        state.retarget();
        state.advance(&second, &springs, dt);

        // Position is continuous: one substep of travel, nothing more.
        let travelled = before.angle_between(state.current[Bone::LeftArm]);
        assert!(
            travelled < velocity_before.length() * SUBSTEP_SECONDS * 2.0 + 1.0e-3,
            "redirecting must not jump the bone: it moved {travelled} rad in one step \
             while travelling at {} rad/s",
            velocity_before.length(),
        );

        // Velocity is continuous: it bends toward the new target rather than
        // resetting. A crossfade would discard it here.
        let velocity_after = state.velocity[Bone::LeftArm];
        let change = (velocity_after - velocity_before).length();
        assert!(
            change < velocity_before.length() * 0.5,
            "redirecting must preserve momentum, but velocity changed by {change} from \
             {velocity_before:?} to {velocity_after:?}",
        );
    }

    #[test]
    fn settled_on_starts_already_in_the_pose_with_no_motion() {
        let pose = pose_with(Bone::RightArm, Quat::from_axis_angle(Vec3::X, 0.9));
        let state = DhoState::settled_on(&pose);

        assert_eq!(state.current[Bone::RightArm], pose.rotation(Bone::RightArm));
        assert!(state.is_settled(1.0e-6), "a settled state must have no velocity");

        // And it must not lurch on the first frame.
        let mut state = state;
        run(&mut state, &pose, 0.5);
        assert!(
            rotation_mismatch(state.current[Bone::RightArm], pose.rotation(Bone::RightArm))
                < 1.0e-9,
            "a rig spawned in a pose must not spring away from it",
        );
    }

    #[test]
    fn a_slacker_spring_lags_a_stiffer_one() {
        // The authoring contract that replaces hand-keyed secondary motion:
        // the same pose data reads as heavy or snappy purely by tuning.
        let target = pose_with(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, FRAC_PI_2));

        let mut stiff_springs = BoneSet::splat(SpringParams::critical(0.05));
        let mut slack_springs = BoneSet::splat(SpringParams::critical(0.30));
        stiff_springs[Bone::LeftArm] = SpringParams::critical(0.05);
        slack_springs[Bone::LeftArm] = SpringParams::critical(0.30);

        let mut stiff = DhoState::AT_REST;
        let mut slack = DhoState::AT_REST;
        for _ in 0..12 {
            stiff.advance(&target, &stiff_springs, 1.0 / 60.0);
            slack.advance(&target, &slack_springs, 1.0 / 60.0);
        }

        let stiff_progress = stiff.current[Bone::LeftArm].angle_between(Quat::IDENTITY);
        let slack_progress = slack.current[Bone::LeftArm].angle_between(Quat::IDENTITY);

        assert!(
            stiff_progress > slack_progress * 1.5,
            "a stiffer spring must visibly lead a slacker one: {stiff_progress} vs \
             {slack_progress} rad after 0.2s",
        );
    }

    #[test]
    fn an_underdamped_bone_overshoots_its_target() {
        // Follow-through, emergent rather than authored.
        let target = pose_with(Bone::LeftHand, Quat::from_axis_angle(Vec3::Z, 1.0));
        let springs = BoneSet::splat(SpringParams {
            halflife: 0.12,
            damping_ratio: 0.3,
            max_speed: 50.0,
        });

        let mut state = DhoState::AT_REST;
        let mut furthest = 0.0f32;
        for _ in 0..180 {
            state.advance(&target, &springs, 1.0 / 120.0);
            furthest =
                furthest.max(state.current[Bone::LeftHand].angle_between(Quat::IDENTITY));
        }

        assert!(
            furthest > 1.0 + 0.02,
            "an underdamped bone should swing past its 1.0 rad target, peaked at {furthest}",
        );
        assert!(
            rotation_mismatch(state.current[Bone::LeftHand], target.rotation(Bone::LeftHand))
                < 1.0e-3,
            "and must still settle on it",
        );
    }

    #[test]
    fn a_zero_or_negative_frame_time_is_a_no_op() {
        let target = pose_with(Bone::Head, Quat::from_axis_angle(Vec3::Y, 0.5));
        let springs = default_springs();

        let mut state = DhoState::AT_REST;
        state.advance(&target, &springs, 0.0);
        state.advance(&target, &springs, -1.0);

        assert_eq!(state, DhoState::AT_REST, "a non-positive dt must change nothing");
    }

    #[test]
    fn every_bone_can_be_driven_independently() {
        // A whole-body pose must animate every bone, not just the ones a
        // test happened to name.
        let mut target = LocalPose::REST;
        for (i, &bone) in Bone::ALL.iter().enumerate() {
            let axis = match i % 3 {
                0 => Vec3::X,
                1 => Vec3::Y,
                _ => Vec3::Z,
            };
            target.set_rotation(bone, Quat::from_axis_angle(axis, 0.3));
        }

        let mut state = DhoState::AT_REST;
        run(&mut state, &target, 3.0);

        for &bone in Bone::ALL.iter() {
            assert!(
                rotation_mismatch(state.current[bone], target.rotation(bone)) < 1.0e-5,
                "{} did not reach its target",
                bone.name(),
            );
        }
    }
}
