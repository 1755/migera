//! Root motion: turning a walk cycle into travel.
//!
//! # The velocity is an output; the position is someone else's
//!
//! There are two obvious ways to make a walking character move, and both
//! have a well-known failure:
//!
//! - **Translate the entity at an authored speed.** The feet then slide,
//!   because the animation was authored for one speed and is being played at
//!   another. Foot locking papers over it, but the error is real and grows
//!   with the mismatch.
//! - **Let the planted foot drive the entity's position.** No sliding by
//!   construction — and now the animation owns the transform, so a character
//!   controller has nothing left to write. Walls, slopes, knockback and
//!   scripted moves all have to fight it.
//!
//! The conflict is not inherent. It comes from conflating two quantities:
//!
//! > **The animation produces a velocity. The controller owns the position.**
//!
//! Each frame the gait measures how far the stance foot moves relative to
//! the body and publishes the negation as a velocity *request*. A controller
//! integrates it — or clamps it against a wall, projects it onto a slope,
//! scales it, or ignores it entirely.
//!
//! **Zero slide by construction.** A foot's world velocity is the body's
//! plus the foot's body-relative velocity. Moving the body at exactly the
//! negation of the stance foot's relative motion cancels them to zero —
//! algebraically, not approximately, so it is testable as an identity rather
//! than a tolerance.
//!
//! **No controller conflict.** Nothing here writes a `Transform`. When a
//! controller refuses the request the feet *do* slide, and that is correct:
//! the character is being dragged. [`super::footlock`] then absorbs exactly
//! that discrepancy, which is its job. The two compose instead of competing.
//!
//! # Measured, not derived from the stride
//!
//! The velocity comes from forward kinematics on the posed skeleton, not
//! from [`super::gait::foot_offset_z`]. The two disagree — the authored
//! offset spans a 0.45 m stride while the posed foot actually travels 0.61 m
//! — because the offset describes intent and the pose describes what the
//! rotations do. Only the second one is what the foot will really do, and
//! cancelling anything else leaves residual slide.

use bevy::math::Vec3;

use super::gait::{leg_phase, walk_pose_on, GaitParams};
use super::rig::{LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// What a character does with the velocity its gait publishes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RootMotion {
    /// Publish it and let a controller integrate it.
    ///
    /// The mode a game uses: the controller can accept, clamp, project or
    /// ignore the request, and whatever it does the foot lock absorbs the
    /// difference.
    #[default]
    Drive,
    /// Integrate it here. The no-controller default, and what a gallery or
    /// a test wants.
    Authoritative,
    /// Publish it but never integrate — a treadmill. The character walks on
    /// the spot, which is what a turntable view wants.
    InPlace,
}

/// One character's locomotion state.
#[derive(Debug, Clone, Copy)]
pub struct Locomotion {
    /// How far the character has travelled, world space. Only
    /// [`RootMotion::Authoritative`] advances this.
    pub position: Vec3,
    /// The velocity the gait is asking for, metres per second. An **output**
    /// — rewritten every frame from the stance foot's own motion.
    pub root_velocity: Vec3,
    /// What to do with it.
    pub mode: RootMotion,
}

impl Default for Locomotion {
    fn default() -> Self {
        Self { position: Vec3::ZERO, root_velocity: Vec3::ZERO, mode: RootMotion::default() }
    }
}

/// Which foot the body should measure its travel against, at a point in the
/// cycle.
///
/// Returns `None` when neither foot is planted — during a run's flight
/// phase, say — because there is then no contact to derive travel from.
///
/// # Picking the foot with the most stance LEFT
///
/// During double support both feet are down and either could serve. The
/// choice is not arbitrary: pick the one about to lift and the reference
/// switches feet within a frame, and a finite difference across that switch
/// measures the *gap between the two feet* rather than one foot's motion.
///
/// Measured with the opposite (and superficially reasonable) rule — follow
/// whichever foot has been planted longer — that produced velocity spikes of
/// **245 m/s** at phases 0.100 and 0.600, with a 110 m/s lateral component
/// that gave the switch away: one foot's motion has no sideways term at all,
/// but the step from one foot to the other is exactly the hip width.
///
/// Choosing by stance REMAINING means the reference only changes when the
/// current foot is genuinely done, and then to a foot that has a full
/// contact ahead of it.
pub fn stance_foot(phase: f32, params: &GaitParams) -> Option<Bone> {
    use super::gait::LegPhase;

    let remaining = |leg: LegPhase| match leg {
        LegPhase::Stance { progress } => Some(1.0 - progress),
        LegPhase::Swing { .. } => None,
    };

    let left = remaining(leg_phase(phase, params.duty_factor));
    let right = remaining(leg_phase(phase + 0.5, params.duty_factor));

    match (left, right) {
        (Some(l), Some(r)) => {
            Some(if l >= r { Bone::LeftFoot } else { Bone::RightFoot })
        }
        (Some(_), None) => Some(Bone::LeftFoot),
        (None, Some(_)) => Some(Bone::RightFoot),
        (None, None) => None,
    }
}

/// Where the stance foot sits relative to the body, at a point in the cycle.
///
/// See [`stance_foot`] for which foot that is.
pub fn stance_foot_offset(
    phase: f32,
    params: &GaitParams,
    base: &LocalPose,
    rig: &RigGeometry,
) -> Option<Vec3> {
    let bone = stance_foot(phase, params)?;
    offset_of(bone, phase, &walk_on(params, base, rig), rig)
}

/// The gait's own pose at a phase: `walk_pose_on` with the rest held fixed.
///
/// `walk_pose_on`, not `walk_pose`: the published velocity is measured from
/// how far the stance foot travels under the body, so it has to measure the
/// pose the character is ACTUALLY in. Posing on the synthetic rig while the
/// character is driven on a real one measures a different animation.
fn walk_on<'a>(
    params: &'a GaitParams,
    base: &'a LocalPose,
    rig: &'a RigGeometry,
) -> impl Fn(f32) -> LocalPose + 'a {
    move |phase| walk_pose_on(phase, params, base, rig)
}

/// One named foot's position relative to the body, in the pose `pose_at`
/// gives at `phase`.
fn offset_of(
    bone: Bone,
    phase: f32,
    pose_at: &dyn Fn(f32) -> LocalPose,
    rig: &RigGeometry,
) -> Option<Vec3> {
    Some(super::rig::offset_from(&pose_at(phase), rig, Bone::Hips, bone))
}

/// The velocity the body must travel at for the stance foot to stay still.
///
/// Measured by finite difference across `dt` of the cycle, so it reflects
/// what the pose actually does rather than what the stride says it should.
///
/// Returns zero where there is no stance foot to derive it from.
pub fn root_velocity(
    phase: f32,
    cadence_hz: f32,
    params: &GaitParams,
    base: &LocalPose,
    rig: &RigGeometry,
) -> Vec3 {
    root_velocity_of(phase, cadence_hz, params, &walk_on(params, base, rig), rig)
}

/// [`root_velocity`], measured on whatever pose the character is ACTUALLY
/// rendered in.
///
/// # Zero slide needs the exact pose and the exact clock
///
/// The cancellation this module rests on holds only if the velocity is
/// derived from the pose the renderer draws, cycling at the rate its phase
/// really advances. The gallery broke both, three ways at once:
///
/// - it measured the gait on the authored base while rendering it on the
///   stance with the knee bend applied;
/// - it rendered a BLEND toward standing during starts and stops, which
///   scales the foot's travel, and published the full-weight velocity;
/// - its legs cycled at the phase clock's `1/7 + 0.9·v` Hz while the
///   velocity assumed `0.9·v` — and the speed slider never reached the leg
///   clock at all, so after moving it the legs kept the launch speed's
///   rhythm. The planted foot slid by `(1/7) / (0.9·v + 1/7)`: 14% at
///   1 m/s, 35% at 0.3.
///
/// `pose_at` is the rendered pose as a function of phase — blend and all —
/// and `cadence_hz` must be the rate that phase actually advances.
pub fn root_velocity_of(
    phase: f32,
    cadence_hz: f32,
    params: &GaitParams,
    pose_at: &dyn Fn(f32) -> LocalPose,
    rig: &RigGeometry,
) -> Vec3 {
    if cadence_hz <= 0.0 {
        return Vec3::ZERO;
    }

    // A small step in CYCLE space. Taken symmetrically about `phase` so the
    // estimate is centred rather than lagging half a step behind.
    const STEP: f32 = 1.0e-3;

    // The foot is chosen ONCE, at the centre, and both samples measure that
    // same foot.
    //
    // Re-selecting per sample is the obvious way to write this and it is
    // wrong: near a contact switch the two samples pick different feet, so
    // the difference measures the gap BETWEEN the feet rather than one
    // foot's motion. That produced 245 m/s velocity spikes — with a 110 m/s
    // lateral component, which is exactly the hip width and was the clue.
    let Some(_) = stance_foot(phase, params) else {
        // No foot down — a run's flight phase. The body is a projectile
        // here: it keeps the velocity it left the ground with.
        //
        // Returning zero instead is what the walk-only code did, and for a
        // run that is a dead stop twice per cycle. At a 0.4 duty factor
        // flight is 20% of the cycle, so at 1.5 strides/s the body would
        // stall for 67 ms, twice a second.
        //
        // Evaluated rather than remembered, which keeps this a pure
        // function of phase — the determinism the whole stack relies on.
        return toe_off_velocity(phase, cadence_hz, params, pose_at, rig);
    };

    // The difference window must lie INSIDE the contact.
    //
    // A centred window at the last grounded instant straddles toe-off: one
    // side is in stance, the other airborne and barely moving relative to
    // the body, which halves the estimate. Measured exactly 1.99x — 2.746
    // m/s inside stance against 1.378 at its edge — and it reads as the
    // body losing half its speed the instant a foot lifts.
    //
    // A walk never showed this because its stance windows overlap, so
    // there is always another foot down to take the reference. A run's do
    // not.
    //
    // # Every planted foot, by the load it carries
    //
    // The body rides on whichever foot bears its weight, and through double
    // support that is both, handing over gradually. Following one foot and
    // switching to the other mid-way is only continuous if the two feet
    // agree exactly — and a measured stride on a rig proportioned unlike
    // the person measured does not quite: the switch became a step in the
    // body's speed. So each planted foot's motion is weighted by
    // `gait::stance_load`, the same ramp that sets the hip height.
    //
    // # The contact point, not the ankle
    //
    // A foot rolls heel to toe (see `super::foot`), so the ankle moves while
    // the foot is planted: forward and down about the heel after contact, up
    // and forward about the ball as the heel rises. What stays put is the
    // loaded end. Its DISPLACEMENT is taken, rather than the displacement of
    // a blended point, because the blend itself slides from heel to ball
    // along the foot as the load hands over — a centre of pressure moves,
    // the body does not.
    use super::foot::{contact_moved, Sole};
    use super::gait::{stance_load, LegPhase};

    const OFFSETS: [f32; 5] = [-2.0 * STEP, -STEP, 0.0, STEP, 2.0 * STEP];
    let mut poses: [Option<LocalPose>; 5] = [None; 5];
    let mut pose_at_offset = |i: usize| *poses[i].get_or_insert_with(|| pose_at(phase + OFFSETS[i]));

    let (mut loads, mut heights, mut motions) = ([0.0f32; 2], [f32::MAX; 2], [None; 2]);
    for (side, (shift, ankle)) in [(0.0, Bone::LeftFoot), (0.5, Bone::RightFoot)].into_iter().enumerate() {
        let in_stance = |p: f32| leg_phase(p + shift, params.duty_factor).is_stance();
        let LegPhase::Stance { progress } = leg_phase(phase + shift, params.duty_factor) else {
            continue;
        };
        let (a, b) = if in_stance(phase - STEP) && in_stance(phase + STEP) {
            (1, 3)
        } else if in_stance(phase - 2.0 * STEP) {
            // Near the END of this foot's stance: look backward only.
            (0, 2)
        } else {
            // Near the START: look forward only.
            (2, 4)
        };

        let sole = Sole::of(rig, ankle);
        let moved = contact_moved(&sole.points(&pose_at_offset(a), rig), &sole.points(&pose_at_offset(b), rig))
            / (OFFSETS[b] - OFFSETS[a]);

        let centre = pose_at_offset(2);
        loads[side] = stance_load(progress, params.duty_factor);
        heights[side] = centre.root_translation.y + super::foot::lowest(&sole.points(&centre, rig));
        motions[side] = Some(moved);
    }
    let feet: Vec<Vec3> = motions.iter().flatten().copied().collect();
    if feet.is_empty() {
        return Vec3::ZERO;
    }
    // Only feet that touch the ground carry the body; see `foot::bearing`.
    let bearing = super::foot::bearing(loads, heights);
    let (sum, total) = (0..2).fold((Vec3::ZERO, 0.0), |(s, t), i| match motions[i] {
        Some(m) => (s + m * bearing[i], t + bearing[i]),
        None => (s, t),
    });
    // At the instant of a lone footfall its load is still zero: a run has
    // no other foot down to carry the reference.
    let per_cycle = if total > 1.0e-6 { sum / total } else { feet.iter().sum::<Vec3>() / feet.len() as f32 };

    // How fast the foot moves relative to the body, per second.
    let relative = per_cycle * cadence_hz;

    // A sanity ceiling on what a leg can produce.
    //
    // Not defensive clutter: `leg_phase` clamps a degenerate duty factor to
    // 0.01 rather than rejecting it, so "no contact at all" becomes a 1%
    // stance sliver. Differentiating across that sliver — where the foot
    // effectively teleports between stance and swing — measured **156 m/s**.
    // A published velocity that large would fling a character across the
    // level in a frame.
    //
    // Expressed against the leg's own reach and the cadence, so it scales
    // with the rig rather than being a magic number: a foot cannot travel
    // more than the leg is long within one stride.
    const IMPLAUSIBLE_STRIDES_PER_SECOND: f32 = 4.0;

    let ceiling = (rig.offsets[Bone::LeftLeg].length()
        + rig.offsets[Bone::LeftFoot].length())
        * cadence_hz
        * IMPLAUSIBLE_STRIDES_PER_SECOND;

    // The body must move the other way for the foot to stand still.
    //
    // The vertical component is deliberately dropped. Two different reasons,
    // and both matter:
    //
    // - Cancelling the foot's own bob would make the body bounce in
    //   antiphase with it, which is not what a walking body does.
    // - Height over terrain is the ground stage's job, not this one's. This
    //   publishes travel ALONG the ground; [`ground_following_height`] is
    //   what puts the character on it.
    //
    // Leaving it at that is what let a character walk 26 m up a 0.3 slope
    // while holding y = 0.949 — 7.7 m underneath the surface. Horizontal
    // travel alone is not locomotion on anything but a flat plane.
    let velocity = Vec3::new(-relative.x, 0.0, -relative.z);

    // `>` is FALSE for a NaN, so a comparison alone lets one through — the
    // clamp has to check finiteness explicitly. Caught by
    // `a_degenerate_gait_cannot_publish_an_absurd_velocity` feeding a NaN
    // duty factor, which `leg_phase` guards but which still poisons the
    // finite difference above.
    if !velocity.is_finite() {
        return Vec3::ZERO;
    }

    if velocity.length() > ceiling {
        velocity.normalize_or_zero() * ceiling
    } else {
        velocity
    }
}

/// How far the body moves between two poses of a gait, for the planted feet
/// to stay where they are: the negation of the planted contacts' motion
/// under the hips, weighted by the load each bears (see
/// [`root_velocity_of`] for why each piece is what it is). Horizontal only.
///
/// `phase` is the cycle position between the two, which decides which feet
/// are planted.
///
/// # A displacement, not a velocity times a step
///
/// Integrating [`root_velocity_of`] with a frame's `dt` is exact only while
/// the velocity holds still across the frame, and a walk's does not: the
/// body slows over each foot and speeds up in double support, about 6 m/s²
/// either way. Sampled at one end of a 60 Hz frame, the planted foot slid
/// up to 2 mm per frame, about a centimetre a stance. The displacement of
/// the contact itself between the two poses is exact for any frame length —
/// and measured on the poses actually RENDERED, it is exact through the
/// springs too, which lag the gait's target by a varying amount.
pub fn root_displacement_between(
    before: &LocalPose,
    after: &LocalPose,
    phase: f32,
    params: &GaitParams,
    rig: &RigGeometry,
) -> Option<Vec3> {
    use super::foot::{bearing, contact_moved, lowest, Sole};
    use super::gait::{stance_load, LegPhase};

    // The contacts are measured from the hips, so a pose that moves its hips
    // by `root_translation` moves every contact with them, unseen. The walk
    // only ever moves them vertically, but the release before a first step
    // shifts them 4.5 cm sideways and leans them 4 cm forward, and fading
    // that out over the first step walked the planted foot 47 mm (headless;
    // ~13 cm live) while root motion saw nothing move.
    let hips_moved = after.root_translation - before.root_translation;
    let hips_moved = Vec3::new(hips_moved.x, 0.0, hips_moved.z);

    let (mut loads, mut heights, mut moved) = ([0.0f32; 2], [f32::MAX; 2], [None; 2]);
    for (side, (shift, ankle)) in [(0.0, Bone::LeftFoot), (0.5, Bone::RightFoot)].into_iter().enumerate() {
        let LegPhase::Stance { progress } = leg_phase(phase + shift, params.duty_factor) else {
            continue;
        };
        let sole = Sole::of(rig, ankle);
        let (a, b) = (sole.points(before, rig), sole.points(after, rig));
        loads[side] = stance_load(progress, params.duty_factor);
        heights[side] = 0.5 * (lowest(&a) + lowest(&b));
        moved[side] = Some(contact_moved(&a, &b) + hips_moved);
    }
    let borne = bearing(loads, heights);
    let (mut sum, mut total, mut plain, mut feet) = (Vec3::ZERO, 0.0, Vec3::ZERO, 0.0);
    for side in 0..2 {
        if let Some(m) = moved[side] {
            sum += m * borne[side];
            total += borne[side];
            plain += m;
            feet += 1.0;
        }
    }
    if feet == 0.0 {
        return None;
    }
    let motion = if total > 1.0e-6 { sum / total } else { plain / feet };
    Some(Vec3::new(-motion.x, 0.0, -motion.z))
}

/// How far the body travels in one full cycle of the gait, metres — the
/// stride as the pose really produces it, not as authored.
///
/// What a caller divides a desired speed by to get the cadence that
/// delivers it: `cadence = speed / distance_per_cycle`. Measured, so a
/// change to any leg curve keeps the speed right automatically.
pub fn distance_per_cycle(params: &GaitParams, base: &LocalPose, rig: &RigGeometry) -> f32 {
    const SAMPLES: usize = 32;
    (0..SAMPLES)
        .map(|i| root_velocity(i as f32 / SAMPLES as f32, 1.0, params, base, rig).length())
        .sum::<f32>()
        / SAMPLES as f32
}

/// The height a character should sit at, given the ground beneath it.
///
/// Root motion publishes travel ALONG the ground, which is all a flat plane
/// needs and not enough for anything else: a character walking a slope on
/// horizontal velocity alone keeps its starting height and sinks into the
/// hillside — measured at 7.7 m under the surface after 26 m of a 0.3 grade.
///
/// So height is sampled rather than integrated. Sampling is also what makes
/// it correct across a step or a ledge, where an integrated vertical
/// velocity would have to guess.
///
/// `stand_height` is how far the character's origin sits above the surface
/// it stands on — the rig's own hip height. Returns `None` where there is no
/// ground, leaving the caller to decide (fall, hover, keep the last value).
pub fn ground_following_height(
    position: Vec3,
    stand_height: f32,
    ground: &dyn super::ground::GroundProbe,
) -> Option<f32> {
    ground.sample(position).map(|hit| hit.height + stand_height)
}

/// The velocity the body had when the last foot left the ground.
///
/// Used through a flight phase, where there is no contact to derive a
/// velocity from and the body is simply coasting.
///
/// Found by walking BACK from `phase` to the moment stance ended, then
/// sampling there. That is a search rather than a formula because the
/// stance windows depend on the duty factor and on which foot — and a
/// formula would need rederiving every time either changed.
fn toe_off_velocity(
    phase: f32,
    cadence_hz: f32,
    params: &GaitParams,
    pose_at: &dyn Fn(f32) -> LocalPose,
    rig: &RigGeometry,
) -> Vec3 {
    const STEPS: usize = 64;

    for back in 1..=STEPS {
        let grounded = phase - back as f32 / STEPS as f32;

        if stance_foot(grounded, params).is_some() {
            return root_velocity_of(grounded, cadence_hz, params, pose_at, rig);
        }
    }

    // A whole cycle with no contact anywhere: not a gait, so there is no
    // travel to infer.
    Vec3::ZERO
}

/// The published velocity, expressed in the world rather than in the
/// character's own frame.
///
/// [`root_velocity`] is derived from hip-relative foot motion, so it is a
/// vector in the CHARACTER's frame: "forward" there means the character's
/// forward, not the world's `-Z`. Integrating it directly into a world
/// position works only for a character that never turns, which is what the
/// gallery did before there was a facing.
///
/// Rotating it by the heading is the whole of root-motion turning.
pub fn world_root_velocity(
    facing: &super::facing::Facing,
    phase: f32,
    cadence_hz: f32,
    params: &GaitParams,
    base: &LocalPose,
    rig: &RigGeometry,
) -> Vec3 {
    facing.rotation() * root_velocity(phase, cadence_hz, params, base, rig)
}

/// Advances one character's locomotion by `dt`.
///
/// Publishes [`Locomotion::root_velocity`] whatever the mode, and integrates
/// it only under [`RootMotion::Authoritative`].
pub fn advance(
    locomotion: &mut Locomotion,
    phase: f32,
    cadence_hz: f32,
    params: &GaitParams,
    base: &LocalPose,
    rig: &RigGeometry,
    dt: f32,
) {
    advance_turning(
        locomotion,
        &mut super::facing::Facing::default(),
        phase,
        cadence_hz,
        params,
        base,
        rig,
        dt,
    );
}

/// [`advance`], for a character that is also turning.
///
/// Turns the facing toward its target, then publishes the velocity in WORLD
/// space so the character travels along its new heading rather than its old
/// one. Returns the [`super::footlock::Turn`] the same frame's foot locks
/// need, so a planted foot pivots with the body instead of being dragged.
///
/// Taking the facing by `&mut` rather than turning it elsewhere keeps the
/// ordering right: the heading must advance BEFORE the velocity is rotated
/// by it, or the character travels one frame behind where it is pointing.
#[allow(clippy::too_many_arguments)]
pub fn advance_turning(
    locomotion: &mut Locomotion,
    facing: &mut super::facing::Facing,
    phase: f32,
    cadence_hz: f32,
    params: &GaitParams,
    base: &LocalPose,
    rig: &RigGeometry,
    dt: f32,
) -> super::footlock::Turn {
    advance_turning_with(
        locomotion,
        facing,
        phase,
        cadence_hz,
        params,
        &walk_on(params, base, rig),
        rig,
        dt,
    )
}

/// [`advance_turning`], deriving the velocity from the pose the character
/// is actually rendered in — see [`root_velocity_of`] for why that, and the
/// real rate its phase advances as `cadence_hz`, are what zero slide needs.
#[allow(clippy::too_many_arguments)]
pub fn advance_turning_with(
    locomotion: &mut Locomotion,
    facing: &mut super::facing::Facing,
    phase: f32,
    cadence_hz: f32,
    params: &GaitParams,
    pose_at: &dyn Fn(f32) -> LocalPose,
    rig: &RigGeometry,
    dt: f32,
) -> super::footlock::Turn {
    let before = facing.yaw;
    facing.advance(dt);
    let yaw_delta = super::facing::shortest_angle(facing.yaw - before);

    locomotion.root_velocity =
        facing.rotation() * root_velocity_of(phase, cadence_hz, params, pose_at, rig);

    // The body's travel this frame, WORLD axes (the IK stage turns it into
    // the pose's for the foot locks). Zero in `Drive` mode: there the body
    // is moved later, after the springs, and whatever moves it fills this in.
    let mut travel = Vec3::ZERO;
    if locomotion.mode == RootMotion::Authoritative && dt > 0.0 {
        // The exact displacement over the frame where a foot is planted (see
        // `root_displacement_between`); the velocity through a flight phase,
        // where the body coasts.
        let started = phase - cadence_hz * dt;
        travel = match root_displacement_between(
            &pose_at(started),
            &pose_at(phase),
            phase - 0.5 * cadence_hz * dt,
            params,
            rig,
        ) {
            Some(moved) => facing.rotation() * moved,
            None => locomotion.root_velocity * dt,
        };
        locomotion.position += travel;
    }

    super::footlock::Turn { pivot: locomotion.position, yaw_delta, travel }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::rig::forward_kinematics_on;
    use crate::character::anim::stance::stance;

    /// A rig with the real asset's leg lengths — parsed from
    /// `puppet_base.gltf` rather than transcribed, because a hand-copied
    /// constant drifts from its source silently — on the synthetic rig,
    /// walking the HAND-SHAPED
    /// curves: the root-motion machinery is gait-agnostic, and these tests
    /// pin it on the curves they were written against. Recorded angles need
    /// a real foot and real leg segments to mean anything — the measured
    /// walk is tested on `puppet_base` ([`real_walk`]).
    fn setup() -> (LocalPose, GaitParams, RigGeometry) {
        (
            stance(&LocalPose::REST),
            GaitParams::authored_walk(),
            crate::character::anim::gltf_rig::real_leg_lengths(),
        )
    }

    /// Where a foot sits in the WORLD, given a body that has travelled
    /// `body_z`.
    fn world_foot_z(
        phase: f32,
        body_z: f32,
        params: &GaitParams,
        base: &LocalPose,
        rig: &RigGeometry,
        bone: Bone,
    ) -> f32 {
        let positions = forward_kinematics_on(&walk_pose_on(phase, params, base, rig), rig);
        body_z + positions[bone].z - positions[Bone::Hips].z
    }

    /// How far a foot's LOADED contact moves in the world between two poses,
    /// for a body that moved by `body_moved` meanwhile.
    ///
    /// The ankle is the wrong thing to hold still: a foot rolls heel to toe
    /// through stance (see `foot`), so the ankle rightly travels while the
    /// foot is planted. What must not move is whichever end bears the load —
    /// taken, like root motion, as the share-weighted DISPLACEMENT of heel
    /// and ball, not the displacement of a blended point.
    fn contact_moved(
        before: &LocalPose,
        after: &LocalPose,
        body_moved: Vec3,
        rig: &RigGeometry,
        ankle: Bone,
    ) -> Vec3 {
        use crate::character::anim::foot::Sole;
        let sole = Sole::of(rig, ankle);
        let moved = crate::character::anim::foot::contact_moved(&sole.points(before, rig), &sole.points(after, rig))
            + body_moved;
        Vec3::new(moved.x, 0.0, moved.z)
    }

    /// The real character as the gallery composes it: `puppet_base`, its
    /// relaxed stance with the standing knee bend applied on the rig.
    fn real_walk() -> (LocalPose, GaitParams, RigGeometry) {
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
        let rig = crate::character::anim::gltf_rig::puppet_base();
        let stood =
            stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        (stood, GaitParams::default(), rig)
    }

    #[test]
    fn a_faster_walk_takes_longer_strides_and_quicker_steps() {
        // A real walker lengthens their stride AND quickens their cadence
        // as they speed up — cadence roughly as speed^0.35, stride as
        // speed^0.65. With a fixed stride, only the cadence can change: a
        // slow walk became slow motion and a fast one a scurry.
        let (stood, _, rig) = real_walk();
        let cycle = |speed: f32| {
            let params = GaitParams::walking_at(speed);
            let distance = distance_per_cycle(&params, &stood, &rig);
            (distance, speed / distance)
        };
        let (slow_stride, slow_cadence) = cycle(0.5);
        let (fast_stride, fast_cadence) = cycle(1.5);

        assert!(
            fast_stride > slow_stride * 1.6,
            "tripling the speed should lengthen the stride well past 1.6x: {slow_stride:.2} -> \
             {fast_stride:.2} m",
        );
        assert!(
            (1.2..2.0).contains(&(fast_cadence / slow_cadence)),
            "and quicken the cadence, by less than the speed: {slow_cadence:.2} -> \
             {fast_cadence:.2} strides/s",
        );
    }

    #[test]
    fn a_walking_body_speeds_up_in_double_support_and_slows_over_the_foot() {
        // A walk is not a constant-speed glide. The body vaults over each
        // stance leg like an inverted pendulum (Winter 6.2.1, 11.3.1),
        // trading speed for height: slowest as it passes over the foot,
        // fastest in double support. Winter's pelvis (greater trochanter,
        // Table A.2(a)) swings 0.73-1.36 of its mean speed over the stride;
        // his trunk 0.84-1.21.
        //
        // The ±25% "surge" this once had was not that — a clock mismatch put
        // it in the wrong place; see `CHARACTER_PROGRESS.md`. The rhythm here
        // is emergent, from the recorded leg angles and root motion's
        // cancellation of the planted foot.
        let (stood, _, rig) = real_walk();
        for speed in [0.7, 1.2, 1.6] {
            let params = GaitParams::walking_on(speed, &rig);
            let speeds: Vec<(f32, f32)> = (0..100)
                .map(|i| {
                    let phase = i as f32 / 100.0;
                    (phase, root_velocity(phase, 1.0, &params, &stood, &rig).length())
                })
                .collect();
            let mean = speeds.iter().map(|s| s.1).sum::<f32>() / speeds.len() as f32;
            let (slow_at, slowest) =
                speeds.iter().copied().fold((0.0, f32::MAX), |a, s| if s.1 < a.1 { s } else { a });
            let (fast_at, fastest) =
                speeds.iter().copied().fold((0.0, f32::MIN), |a, s| if s.1 > a.1 { s } else { a });
            // Where: slowest in single support, fastest within double
            // support or at its edge (the left foot lands at 0, the right at
            // 0.5; each stance lasts 0.609).
            let single = |p: f32| (0.11..0.5).contains(&(p % 0.5 + if p % 0.5 < 0.11 { 0.5 } else { 0.0 }));
            let double = |p: f32| (p % 0.5) < 0.16 || (p % 0.5) > 0.47;
            assert!(single(slow_at), "at {speed} m/s slowest at {slow_at}, not over the foot");
            assert!(double(fast_at), "at {speed} m/s fastest at {fast_at}, not in double support");
            // How much: the recording's pelvis, within a margin for a rig
            // proportioned differently — and wider for a slow walk. The swing
            // is kinetic energy traded for height, `Δ(v²/2) = g·Δh`, and the
            // bob shrinks with speed more slowly than `v²` does, so the
            // relative swing grows as a walk slows: about `v^-0.7`, 1.6x
            // Winter's at half his speed. Measured 0.73..1.53 at 0.7 m/s.
            assert!(
                (0.6..0.92).contains(&(slowest / mean)) && (1.1..1.65).contains(&(fastest / mean)),
                "at {speed} m/s the body's speed spans {:.2}..{:.2} of its mean; Winter's pelvis \
                 spans 0.73..1.36",
                slowest / mean,
                fastest / mean,
            );
        }
    }

    #[test]
    fn a_planted_foot_stays_on_the_ground() {
        // The hips' height used to be an authored curve that knew nothing
        // about where the stance foot was, so the planted foot was pushed
        // into the ground or lifted off it through every stance. Derived
        // from the stance foot, it stays on the floor — its LOADED contact,
        // heel, ball or toe tip as the foot rolls: the ankle itself rises
        // 12 cm by toe-off, as Winter's does (Table A.2(c)).
        //
        // Not EXACTLY on it since the pelvis rides a smoothed path
        // (`walk::BOB_HARMONICS`): the raw path the feet ask for dropped the
        // body onto each leg. Bounded differently each way. A foot pressed
        // into the floor is lifted by the foot IK, which only has to bend a
        // knee. A floating one would need more leg than the rig has at
        // midstance (`rig_authored_at_critical_extension`), and the IK would
        // drop the pelvis for it, which is the lump back again.
        use crate::character::anim::foot::{lowest, Sole};
        let (stood, _, rig) = real_walk();

        for speed in [0.7, 1.2, 1.6] {
            let params = GaitParams::walking_on(speed, &rig);
            let (mut floating, mut pressed) = (0.0f32, 0.0f32);
            for i in 0..64 {
                let phase = i as f32 / 64.0;
                let pose = walk_pose_on(phase, &params, &stood, &rig);
                let risen = pose.root_translation.y - stood.root_translation.y;
                // Single support only: there the hips answer to one foot.
                for (shift, ankle) in [(0.0, Bone::LeftFoot), (0.5, Bone::RightFoot)] {
                    if let crate::character::anim::gait::LegPhase::Stance { progress } =
                        crate::character::anim::gait::leg_phase(phase + shift, params.duty_factor)
                        && (0.25..=0.75).contains(&progress)
                    {
                        let sole = Sole::of(&rig, ankle);
                        let height = risen + lowest(&sole.points(&pose, &rig)) - lowest(&sole.points(&stood, &rig));
                        floating = floating.max(height);
                        pressed = pressed.max(-height);
                    }
                }
            }
            assert!(
                floating < 0.002,
                "at {speed} m/s a planted foot's contact floated {:.1} mm above the ground",
                floating * 1000.0,
            );
            // Measured 15.3 / 10.8 / 7.3 mm at these speeds: the foot IK
            // lifts it by bending the stance knee at midstance, at the slow
            // walk from ~7 to ~22 degrees — Winter's midstance knee is
            // 15-20.
            assert!(
                pressed < 0.016,
                "at {speed} m/s a planted foot's contact sank {:.1} mm into the ground",
                pressed * 1000.0,
            );
        }
    }

    #[test]
    fn a_swinging_foot_clears_the_ground() {
        // Winter's toe clears the floor by 1.52 cm at its lowest in swing
        // (Problem 3.6-4 on Table A.2(d)); replayed verbatim on this rig's
        // longer toes, the tip brushed it. Never below the floor, and a
        // centimetre clear through mid-swing — `walk::clear_swinging_feet`.
        use crate::character::anim::foot::{lowest, Sole};
        let (stood, _, rig) = real_walk();
        for speed in [0.7, 1.2, 1.6] {
            let params = GaitParams::walking_on(speed, &rig);
            let (mut lowest_ever, mut lowest_mid) = (f32::MAX, f32::MAX);
            for i in 0..200 {
                let phase = i as f32 / 200.0;
                let pose = walk_pose_on(phase, &params, &stood, &rig);
                let risen = pose.root_translation.y - stood.root_translation.y;
                let crate::character::anim::gait::LegPhase::Swing { progress } =
                    crate::character::anim::gait::leg_phase(phase, params.duty_factor)
                else {
                    continue;
                };
                let sole = Sole::of(&rig, Bone::LeftFoot);
                let clear = risen + lowest(&sole.points(&pose, &rig)) - lowest(&sole.points(&stood, &rig));
                lowest_ever = lowest_ever.min(clear);
                if (0.3..0.7).contains(&progress) {
                    lowest_mid = lowest_mid.min(clear);
                }
            }
            // A millimetre or two at the instant before a heel strike, inside
            // the guard's rounded corner (measured 1.3 mm at 1.6 m/s); the
            // runtime foot IK's ground clamp owns contact from there.
            assert!(lowest_ever > -2.0e-3, "at {speed} m/s the swinging foot dips {lowest_ever} m into the floor");
            assert!(lowest_mid > 0.01, "at {speed} m/s mid-swing clears the floor by only {lowest_mid} m");
        }
    }

    #[test]
    fn the_walk_replays_winters_joint_curves() {
        // The walk's legs are Winter's recorded stride (Appendix A); on the
        // real rig, at the speed that stride scales to, they must come back
        // out: the thigh from vertical, the knee, and the foot's attitude on
        // the ground — the three things that are driven. Signed, per sample.
        //
        // Allowed to differ by what the rig forces: the knee is held 6.9
        // degrees short of straight (`gait::KNEE_FLOOR`), the thigh carries
        // the correction that keeps both feet planted (`walk::WalkCycle`),
        // and the swinging toe is lifted clear of the floor.
        use crate::character::anim::gait::{leg_joints, sagittal_angles};
        use crate::character::anim::reference::WINTER;
        let (stood, _, rig) = real_walk();
        let params = GaitParams::walking_on(
            crate::character::anim::reference::SPEED
                * (crate::character::anim::gait::leg_length_of(&rig)
                    / crate::character::anim::reference::LEG_LENGTH)
                    .sqrt(),
            &rig,
        );
        let bind = sagittal_angles(&LocalPose::REST, &rig, leg_joints(Bone::LeftFoot));
        let bind_shank = bind[0] - bind[2];

        let (mut thigh, mut knee, mut foot) = (0.0f32, 0.0f32, 0.0f32);
        let n = 100;
        for i in 0..n {
            let phase = i as f32 / n as f32;
            let [t, _, k, a] = sagittal_angles(&walk_pose_on(phase, &params, &stood, &rig), &rig, leg_joints(Bone::LeftFoot));
            let toe_up = (t - k) + a - bind_shank;
            thigh = thigh.max((t - WINTER.thigh.at(phase)).abs());
            knee += (k - WINTER.knee.at(phase)).powi(2);
            // The foot on the ground: in swing, `walk::clear_swinging_feet`
            // turns it to clear the floor, by design.
            if phase < crate::character::anim::reference::STANCE_FRACTION {
                foot = foot.max((toe_up + WINTER.foot_pitch.at(phase)).abs());
            }
        }
        let knee = (knee / n as f32).sqrt();
        // The thigh's error is the correction that keeps both feet planted
        // through double support; the knee's is the floor, 6.9 degrees held
        // where the recording is straight, a tenth of the stride.
        assert!(thigh.to_degrees() < 2.0, "thigh off Winter's by up to {:.1} degrees", thigh.to_degrees());
        assert!(knee.to_degrees() < 3.0, "knee off Winter's by {:.1} degrees RMS", knee.to_degrees());
        assert!(foot.to_degrees() < 1.0, "a planted foot's attitude off Winter's by up to {:.1} degrees", foot.to_degrees());
    }

    #[test]
    fn a_walking_body_keeps_its_feet_under_it() {
        // Averaged over a stride, Winter's ankle runs 6.5 cm behind his hip
        // marker (Tables A.2(a)/(c)), 4.6% of his stride; his trunk's centre
        // of mass sits within 4 mm of the hip. A foot pattern shifted bodily
        // forward or back reads as leaning over trailing feet — observed
        // live, once, at 25 cm.
        let (stood, _, rig) = real_walk();
        let params = GaitParams::walking_on(1.2, &rig);
        let n = 100;
        let behind: f32 = (0..n)
            .map(|i| {
                let pose = walk_pose_on(i as f32 / n as f32, &params, &stood, &rig);
                let ahead = |bone| crate::character::anim::rig::offset_from(&pose, &rig, Bone::Hips, bone).dot(rig.forward());
                -(ahead(Bone::LeftFoot) + ahead(Bone::RightFoot)) * 0.5
            })
            .sum::<f32>()
            / n as f32;
        let stride = distance_per_cycle(&params, &stood, &rig);
        assert!(
            (0.0..0.09).contains(&(behind / stride)),
            "the ankles average {behind:.3} m behind the hips over a {stride:.2} m stride; \
             Winter's, 4.6% of it",
        );
    }

    #[test]
    fn a_walking_body_rises_and_falls_smoothly() {
        // The pelvis height the planted legs ask for is lumpy on this rig:
        // it fell 14 mm through late single support and was caught at the
        // next heel contact, peaking at 9.5 m per cycle² (~7.7 m/s² at this
        // cadence; live, with the foot IK dropping the pelvis for floating
        // feet, 44 m/s²) — the body dropping onto each leg. The walk now
        // rides one sinusoid per step (`walk::BOB_HARMONICS`): 0.98.
        let (stood, params, rig) = real_walk();
        const N: usize = 480;
        let height: Vec<f32> = (0..N)
            .map(|i| walk_pose_on(i as f32 / N as f32, &params, &stood, &rig).root_translation.y)
            .collect();
        let step = 1.0 / N as f32;
        let worst = (0..N)
            .map(|i| {
                let at = |k: isize| height[(i as isize + k).rem_euclid(N as isize) as usize];
                ((at(1) - 2.0 * at(0) + at(-1)) / (step * step)).abs()
            })
            .fold(0.0f32, f32::max);
        assert!(worst < 2.0, "the pelvis accelerates up to {worst:.2} m per cycle², vertically");
    }

    #[test]
    fn a_walking_body_is_highest_over_its_stance_foot() {
        // Walking vaults over a stiff-ish stance leg: the body rises to its
        // highest at midstance and sinks lowest in double support, when both
        // legs are spread. The authored curve had it backwards — lowest at
        // midstance, which is how a RUN moves.
        let (stood, params, rig) = real_walk();
        let hips_at = |phase: f32| walk_pose_on(phase, &params, &stood, &rig).root_translation.y;

        // The left foot lands at 0; with a 0.6 duty factor the right lifts at
        // 0.1, so 0.05 is double support and 0.3 is left midstance.
        let (double, midstance) = (hips_at(0.05), hips_at(0.3));

        // And by a visible but modest amount. Winter's pelvis centre rises
        // and falls 36 mm (43 scaled to this leg); this one measures 14 mm,
        // emergent from the recorded leg angles on this rig's proportions —
        // imposing the recorded bob instead cost 7-10 degrees of leg angle
        // (see `walk`'s module note). Pinned against a leg-curve change
        // silently making the walk flat or bouncy.
        let samples: Vec<f32> = (0..64).map(|i| hips_at(i as f32 / 64.0)).collect();
        let range = samples.iter().copied().fold(f32::MIN, f32::max)
            - samples.iter().copied().fold(f32::MAX, f32::min);
        assert!(
            (0.01..0.06).contains(&range),
            "the body should rise and fall 1-6 cm per step, not {:.1} mm",
            range * 1000.0,
        );
        assert!(
            midstance > double + 0.005,
            "the hips should be highest at midstance: {:.1} mm there against {:.1} in double \
             support",
            midstance * 1000.0,
            double * 1000.0,
        );
    }

    #[test]
    fn the_default_springs_keep_a_walking_foot_planted() {
        // The rendered pose is the gait's target filtered through the
        // per-bone springs. Moved by the velocity measured on the TARGET,
        // the body out-walked its own planted foot wherever the springs
        // lagged: 302 mm a stance at the old 0.12 s leg half-life (see
        // `dho::default_springs` for the sweep that set the legs to 0.015),
        // and 39 mm even at 0.015 on the measured walk, whose speed rises
        // and falls through every stride so the lag never holds still. The
        // gallery moves the body by the RENDERED contact's displacement
        // (`root_displacement_between`) instead, which this replays.
        //
        // Checked at a brisk cadence as well as a normal one: a low-pass
        // attenuates a faster stride more.
        use crate::character::anim::dho::{default_springs, DhoState};
        use crate::character::anim::gait::{leg_phase, LegPhase};

        let (stood, params, rig) = real_walk();
        let springs = default_springs();
        let dt = 1.0 / 60.0;

        for cadence in [1.0f32, 1.6] {
            let mut phase = 0.0f32;
            let mut state = DhoState::settled_on(&walk_pose_on(phase, &params, &stood, &rig));
            let mut previous = state.pose(Vec3::ZERO);
            let mut slid = 0.0f32;
            let mut worst = 0.0f32;

            for frame in 0..(5.0 / dt) as usize {
                phase += cadence * dt;
                let target = walk_pose_on(phase, &params, &stood, &rig);
                state.advance(&target, &springs, dt);
                let rendered = state.pose(target.root_translation);
                // Root motion as the gallery drives it: the rendered planted
                // contact's own displacement over the frame.
                let body = root_displacement_between(&previous, &rendered, phase - 0.5 * cadence * dt, &params, &rig)
                    .unwrap_or(Vec3::ZERO);

                // Single support on the left foot, after two warm-up
                // seconds: how far its loaded contact slides over it.
                let single = matches!(
                    leg_phase(phase, params.duty_factor),
                    LegPhase::Stance { progress } if (0.2..=0.8).contains(&progress)
                );
                if single && frame as f32 * dt > 2.0 {
                    let step = contact_moved(&previous, &rendered, body, &rig, Bone::LeftFoot).length();
                    slid += step;
                    worst = worst.max(slid);
                } else {
                    slid = 0.0;
                }
                previous = rendered;
            }

            assert!(
                worst < 0.005,
                "at {cadence} strides/s a planted foot slid {:.1} mm under the default springs",
                worst * 1000.0,
            );
        }
    }

    #[test]
    fn a_walking_foot_moves_smoothly_at_any_frame_rate() {
        // The spring advances in whole 1/120 s substeps, but root motion and
        // hip height follow the frame's real time. Rendering the substep
        // state therefore lagged each frame by a varying 0-8.3 ms, and a
        // near-instant leg popped by however far it moved in that slack.
        // Live on a 59.96 Hz display this was a stutter that came and went
        // every ~12.5 s (the slack drifting across a substep boundary, with
        // vsync jitter picking 1, 2 or 3 substeps per frame near it). With
        // the substep state rendered: the largest frame-to-frame change in
        // foot velocity was 3.5 m/s at 59.96 Hz, against 0.68 at an exact
        // 60 Hz, where it is the footfall itself. Rendering the state at the
        // frame's time left 1.27 — a footfall landing in a 3-substep frame,
        // all three chasing the frame-END target — until substeps were given
        // the target at their own moment. Now 0.71 at 59.96 Hz, 0.31 at 144.
        //
        // Replays the gallery's per-frame loop and compares each frame rate
        // against the exact-60 Hz baseline. Runs 20 s so the 59.96 Hz slack
        // crosses a substep boundary.
        use crate::character::anim::dho::{default_springs, DhoState};

        let (stood, params, rig) = real_walk();
        let springs = default_springs();
        let cadence = 1.0f32;

        // Deterministic frame-time jitter in [-0.5, 0.5).
        let mut seed = 12345u32;
        let mut noise = move || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
        };

        let mut worst_change = |frame: f32, jitter: f32| {
            let mut phase = 0.0f32;
            let mut state = DhoState::settled_on(&walk_pose_on(phase, &params, &stood, &rig));
            let mut body = Vec3::ZERO;
            let mut previous: Option<(Vec3, Vec3)> = None;
            let mut worst = 0.0f32;
            let mut t = 0.0f32;
            while t < 20.0 {
                let dt = frame + jitter * noise();
                t += dt;
                phase += cadence * dt;
                let target = walk_pose_on(phase, &params, &stood, &rig);
                state.advance(&target, &springs, dt);
                body += root_velocity(phase, cadence, &params, &stood, &rig) * dt;

                let rendered = state.pose(target.root_translation);
                let foot = body
                    + crate::character::anim::rig::offset_from(
                        &rendered,
                        &rig,
                        Bone::Hips,
                        Bone::LeftFoot,
                    );
                // Per second, so different frame rates compare.
                let velocity = previous.map_or(Vec3::ZERO, |(at, _)| (foot - at) / dt);
                if let Some((_, before)) = previous
                    && t > 1.0
                {
                    worst = worst.max((velocity - before).length());
                }
                previous = Some((foot, velocity));
            }
            worst
        };

        let baseline = worst_change(1.0 / 60.0, 0.0);
        for (label, frame, jitter) in
            [("59.96 Hz, ±0.25 ms", 1.0 / 59.96, 0.0005), ("144 Hz", 1.0 / 144.0, 0.0)]
        {
            let worst = worst_change(frame, jitter);
            assert!(
                worst < baseline * 1.5,
                "at {label} a walking foot's velocity jumped {worst:.2} m/s between frames, \
                 against {baseline:.2} at an exact 60 Hz",
            );
        }
    }

    #[test]
    fn the_measured_walks_velocity_never_jumps() {
        // A step in the published velocity is a visible lurch. The walk's
        // speed rises and falls through every stride (see
        // `a_walking_body_speeds_up_in_double_support_and_slows_over_the_foot`)
        // but continuously: measured 0.053 m/s at most between samples 1/400
        // of a stride apart, at 1 stride/s. On the synthetic rig with real
        // leg lengths but no real foot, the same walk jumped 1.1 m/s at each
        // toe-off — recorded angles need the rig they are recorded for.
        let (stood, _, rig) = real_walk();
        for speed in [0.7, 1.2, 1.6] {
            let params = GaitParams::walking_on(speed, &rig);
            let n = 400;
            let v: Vec<Vec3> =
                (0..=n).map(|i| root_velocity(i as f32 / n as f32, 1.0, &params, &stood, &rig)).collect();
            let worst = (1..=n).map(|i| (v[i] - v[i - 1]).length()).fold(0.0, f32::max);
            assert!(worst < 0.1, "at {speed} m/s the velocity jumped {worst} m/s between samples");
        }
    }

    #[test]
    fn the_default_springs_keep_the_arms_counter_swinging() {
        // Through the springs, whenever one foot is clearly ahead the
        // OPPOSITE hand should be. A 0.12 s arm spring is a ~5.8 rad/s
        // low-pass against a ~5.7 rad/s stride — about 88 degrees of lag,
        // a quarter-cycle — and live the opposite hand led in only 42% of
        // samples, chance level; 90% with the springs made instant.
        use crate::character::anim::dho::{default_springs, DhoState};

        let (stood, params, rig) = real_walk();
        let springs = default_springs();
        let (cadence, dt) = (0.9, 1.0 / 60.0);
        let mut phase = 0.0f32;
        let mut state = DhoState::settled_on(&walk_pose_on(phase, &params, &stood, &rig));
        let ahead = |pose: &LocalPose, bone| {
            crate::character::anim::rig::offset_from(pose, &rig, Bone::Hips, bone).dot(rig.forward())
        };

        let (mut split, mut agreeing) = (0, 0);
        for frame in 0..(6.0 / dt) as usize {
            phase += cadence * dt;
            let target = walk_pose_on(phase, &params, &stood, &rig);
            state.advance(&target, &springs, dt);
            if (frame as f32) * dt < 2.0 {
                continue;
            }
            let rendered = state.pose(target.root_translation);
            let feet = ahead(&rendered, Bone::LeftFoot) - ahead(&rendered, Bone::RightFoot);
            let hands = ahead(&rendered, Bone::RightHand) - ahead(&rendered, Bone::LeftHand);
            if feet.abs() > 0.05 {
                split += 1;
                agreeing += usize::from((feet > 0.0) == (hands > 0.0));
            }
        }

        let share = agreeing as f32 / split as f32;
        assert!(
            share > 0.85,
            "the opposite hand should lead whenever a foot does, but did in {:.0}% of {split} \
             samples",
            share * 100.0,
        );
    }

    // -----------------------------------------------------------------
    // The cancellation identity — the whole point of this module
    // -----------------------------------------------------------------

    #[test]
    fn integrating_the_published_velocity_holds_the_stance_foot_still() {
        // THE property. Not "the foot barely moves" — the body velocity is
        // defined as the negation of the foot's relative motion, so the two
        // cancel algebraically and the residual is integration error alone.
        let (base, params, rig) = setup();
        let cadence = 1.0;
        let dt = 1.0 / 240.0;

        // Single support on the left foot, well away from any hand-over.
        let mut phase = 0.15;
        let mut worst = 0.0f32;
        let mut previous = walk_pose_on(phase, &params, &base, &rig);

        for _ in 0..40 {
            let velocity = root_velocity(phase, cadence, &params, &base, &rig);
            phase += cadence * dt;

            let now = walk_pose_on(phase, &params, &base, &rig);
            let moved = contact_moved(&previous, &now, velocity * dt, &rig, Bone::LeftFoot);
            worst = worst.max(moved.length());
            previous = now;
        }

        assert!(
            worst < 1.0e-4,
            "a planted foot moved {worst} m in one frame while the body tracked its \
             own published velocity — the cancellation is not exact",
        );
    }

    #[test]
    fn a_half_blended_gait_still_holds_its_stance_foot_still() {
        // Starting and stopping render a BLEND toward standing, which
        // shortens the foot's travel. Measured on that blended pose, the
        // velocity still cancels it; measured on the full gait — what the
        // gallery used to publish — the foot slides by the difference.
        let (base, params, rig) = setup();
        let weight = 0.5;
        let blended = |phase: f32| {
            crate::character::anim::clip::blend(&base, &walk_pose_on(phase, &params, &base, &rig), weight)
        };

        // Single support on the left foot; its loaded contact is what must
        // hold still (see `contact_moved`).
        let (cadence, dt) = (1.0, 1.0 / 240.0);
        let mut phase = 0.2;
        let mut previous = blended(phase);
        let (mut worst, mut unblended_worst) = (0.0f32, 0.0f32);
        for _ in 0..40 {
            let velocity = root_velocity_of(phase, cadence, &params, &blended, &rig);
            let full = root_velocity(phase, cadence, &params, &base, &rig);
            phase += cadence * dt;

            let now = blended(phase);
            worst = worst.max(contact_moved(&previous, &now, velocity * dt, &rig, Bone::LeftFoot).length());
            unblended_worst = unblended_worst.max(((velocity - full) * dt).length());
            previous = now;
        }

        assert!(worst < 1.0e-4, "the blended stance foot moved {worst} m in one frame");
        assert!(
            unblended_worst > 1.0e-3,
            "test setup: the full gait's velocity should differ from the blend's, by \
             {unblended_worst} m per frame",
        );
    }

    #[test]
    fn the_cancellation_holds_across_a_whole_cycle_including_contact_switches() {
        // The mid-stance identity test above proves the algebra; this proves
        // it survives a full cycle — both feet, every contact handover, at a
        // realistic frame rate.
        //
        // That distinction matters: a per-instant identity can still
        // accumulate at the seams, which is exactly where the foot-selection
        // rule earns its keep. Measured: 0.0074 m of total tracked-foot
        // travel over a cycle that carries the body ~1.04 m, with a worst
        // single frame of 0.17 mm.
        //
        // Every planted foot is checked through its WHOLE stance, double
        // support included. There the body rides both feet by load (see
        // `root_velocity_of`), so a foot is exact only when it carries all
        // of it and slides by however much the two feet disagree otherwise —
        // and that disagreement is the thing to bound: a stride recorded on
        // one person, replayed on a rig proportioned differently.
        //
        // Both walks: the hand-shaped one on the synthetic rig with real leg
        // lengths, and the measured one on the rig it is recorded for. The
        // body moves by `root_displacement_between` — what `Authoritative`
        // root motion integrates.
        use crate::character::anim::foot::{lowest, Sole};
        let (real_base, _, real_rig) = real_walk();
        let (authored_base, authored, authored_rig) = setup();
        for (name, base, params, rig) in [
            ("authored", authored_base, authored, authored_rig),
            ("measured", real_base, GaitParams::default(), real_rig.clone()),
            ("measured at 0.7 m/s", real_base, GaitParams::walking_on(0.7, &real_rig), real_rig.clone()),
            ("measured at 1.6 m/s", real_base, GaitParams::walking_on(1.6, &real_rig), real_rig),
        ] {
            let frames = 240usize;
            let dt = 1.0 / frames as f32;

            // "Planted" means ON THE GROUND, not merely in the stance window:
            // a heel can meet the floor a little after its footfall on a rig
            // proportioned unlike the recording, and a trailing toe leave it
            // a little before toe-off (see `foot::bearing`).
            // Judged by the same weights root motion and the pelvis use: a
            // foot carries the body when its support share is most of it.
            let soles = [Sole::of(&rig, Bone::LeftFoot), Sole::of(&rig, Bone::RightFoot)];
            let on_ground = |pose: &LocalPose, side: usize, phase: f32| {
                let heights = [0, 1].map(|s| lowest(&soles[s].points(pose, &rig)));
                let loads = [0.0, 0.5].map(|shift| match leg_phase(phase + shift, params.duty_factor) {
                    crate::character::anim::gait::LegPhase::Stance { progress } => {
                        crate::character::anim::gait::stance_load(progress, params.duty_factor)
                    }
                    _ => 0.0,
                });
                crate::character::anim::foot::bearing(loads, heights)[side] > 0.5
            };

            let (mut worst, mut total) = ([0.0f32; 2], [0.0f32; 2]);
            let mut previous = walk_pose_on(0.0, &params, &base, &rig);
            for i in 0..frames {
                let phase = i as f32 / frames as f32;
                let now = walk_pose_on(phase + dt, &params, &base, &rig);
                let body = root_displacement_between(&previous, &now, phase + 0.5 * dt, &params, &rig)
                    .unwrap_or(Vec3::ZERO);
                for (side, (shift, ankle)) in [(0.0, Bone::LeftFoot), (0.5, Bone::RightFoot)].into_iter().enumerate() {
                    let planted = |p: f32| leg_phase(p + shift, params.duty_factor).is_stance();
                    if !(planted(phase) && planted(phase + dt) && on_ground(&previous, side, phase) && on_ground(&now, side, phase + dt)) {
                        continue;
                    }
                    let step = contact_moved(&previous, &now, body, &rig, ankle).length();
                    worst[side] = worst[side].max(step);
                    total[side] += step;
                }
                previous = now;
            }

            // Measured, total slide per stance: authored 0.25 mm; measured
            // 5.9 mm, worst 0.61 mm in one 1/240 step — all in the weight
            // hand-overs, on the foot carrying the smaller share. Before the
            // pelvis height, root motion and the thigh correction shared one
            // set of support weights, the measured walk slid 41 mm, up to 3.9
            // in one step. Holding both feet exactly still is possible (0.03
            // mm) but bent the thigh 4.4 degrees off the recording and put
            // the pelvis lowest at mid-stance, backwards for a walk.
            for side in 0..2 {
                assert!(
                    worst[side] < 0.8e-3,
                    "{name}: a planted foot moved {} m in a single frame — the cancellation \
                     breaks down somewhere in the cycle",
                    worst[side],
                );
                assert!(
                    total[side] < 0.008,
                    "{name}: a planted foot slid {} m over its stance, in a cycle that \
                     carries the body over a metre",
                    total[side],
                );
            }
        }
    }

    #[test]
    fn a_refused_velocity_leaves_the_foot_sliding() {
        // Proves the test above is not vacuous. With the body held still —
        // a controller refusing the request, as one does at a wall — the
        // same foot slides by a large, obvious amount.
        let (base, params, rig) = setup();
        let cadence = 1.0;
        let dt = 1.0 / 240.0;

        let mut phase = 0.15;
        let mut travelled = 0.0f32;
        let mut previous = world_foot_z(phase, 0.0, &params, &base, &rig, Bone::LeftFoot);

        for _ in 0..40 {
            phase += cadence * dt;
            let now = world_foot_z(phase, 0.0, &params, &base, &rig, Bone::LeftFoot);
            travelled += (now - previous).abs();
            previous = now;
        }

        assert!(
            travelled > 0.05,
            "with the body stationary the planted foot should slide visibly, but it \
             only moved {travelled} m — the cancellation test may be vacuous",
        );
    }

    #[test]
    fn the_published_velocity_points_forward() {
        // A walk goes toward -Z. A sign error here is the same class of bug
        // that had the whole gait running backward, so it gets its own
        // assertion rather than being implied by the cancellation.
        let (base, params, rig) = setup();

        for phase in [0.05_f32, 0.15, 0.3, 0.5] {
            let velocity = root_velocity(phase, 1.0, &params, &base, &rig);
            assert!(
                velocity.z < 0.0,
                "at phase {phase} the published velocity is {velocity:?} — the \
                 character would travel backward",
            );
        }
    }

    #[test]
    fn the_speed_scales_with_cadence() {
        let (base, params, rig) = setup();

        let slow = root_velocity(0.15, 1.0, &params, &base, &rig);
        let fast = root_velocity(0.15, 2.0, &params, &base, &rig);

        assert!(
            (fast.z - slow.z * 2.0).abs() < 1.0e-4,
            "doubling the cadence should double the speed: {} vs {}",
            slow.z,
            fast.z,
        );
    }

    #[test]
    fn a_stopped_gait_publishes_no_velocity() {
        let (base, params, rig) = setup();
        assert_eq!(root_velocity(0.15, 0.0, &params, &base, &rig), Vec3::ZERO);
        assert_eq!(root_velocity(0.15, -1.0, &params, &base, &rig), Vec3::ZERO);
    }

    #[test]
    fn the_velocity_has_no_vertical_component() {
        // The body must not rise and fall to cancel the foot's own bob —
        // that is a bouncing character, and height belongs to the ground
        // stage anyway.
        let (base, params, rig) = setup();

        for i in 0..20 {
            let phase = i as f32 / 20.0;
            assert_eq!(
                root_velocity(phase, 1.0, &params, &base, &rig).y,
                0.0,
                "at phase {phase}",
            );
        }
    }

    // -----------------------------------------------------------------
    // Which foot is tracked
    // -----------------------------------------------------------------

    #[test]
    fn a_walk_always_has_a_stance_foot_to_track() {
        // A walk never leaves the ground, so the velocity is defined at
        // every point in the cycle. A gap would show up as a one-frame
        // velocity dropout.
        let (base, params, rig) = setup();

        for i in 0..200 {
            let phase = i as f32 / 200.0;
            assert!(
                stance_foot_offset(phase, &params, &base, &rig).is_some(),
                "no stance foot at phase {phase}",
            );
        }
    }

    #[test]
    fn the_tracked_foot_is_the_one_with_the_most_stance_left() {
        // During double support both feet are down and either could serve.
        // Following the one planted LONGER is the superficially reasonable
        // rule and it is wrong: that foot is about to lift, so the reference
        // switches within a frame and the velocity spikes.
        let (base, params, rig) = setup();

        // Just after the right foot lands at phase 0.5, the left is near the
        // end of its own stance — so the RIGHT, with a full contact ahead of
        // it, is the one to measure against.
        assert_eq!(
            stance_foot(0.52, &params),
            Some(Bone::RightFoot),
            "the freshly-planted foot should take over the reference",
        );

        // And early in the left's stance, before the right lands, it is the
        // left.
        assert_eq!(stance_foot(0.05, &params), Some(Bone::LeftFoot));

        // The offset follows whichever foot that is.
        let offset = stance_foot_offset(0.52, &params, &base, &rig).expect("both down");
        let positions = forward_kinematics_on(&walk_pose_on(0.52, &params, &base, &rig), &rig);
        let right = positions[Bone::RightFoot] - positions[Bone::Hips];

        assert!(
            (offset - right).length() < 1.0e-6,
            "got {offset:?} against a right foot at {right:?}",
        );
    }

    #[test]
    fn the_velocity_never_jumps_discontinuously() {
        // A step change in the published velocity is a visible lurch, and it
        // is what tracking the wrong foot at a contact switch produces.
        let (base, params, rig) = setup();

        let mut previous = root_velocity(0.0, 1.0, &params, &base, &rig);
        let mut worst = 0.0f32;

        for i in 1..=400 {
            let phase = i as f32 / 400.0;
            let now = root_velocity(phase, 1.0, &params, &base, &rig);
            worst = worst.max((now - previous).length());
            previous = now;
        }

        assert!(
            worst < 0.5,
            "the published velocity jumped by {worst} m/s between adjacent samples",
        );
    }

    // -----------------------------------------------------------------
    // Modes
    // -----------------------------------------------------------------

    #[test]
    fn authoritative_mode_integrates_the_velocity_itself() {
        let (base, params, rig) = setup();

        let mut locomotion =
            Locomotion { mode: RootMotion::Authoritative, ..Default::default() };

        for i in 0..120 {
            advance(
                &mut locomotion,
                i as f32 / 120.0,
                1.0,
                &params,
                &base,
                &rig,
                1.0 / 120.0,
            );
        }

        assert!(
            locomotion.position.z < -0.2,
            "a full cycle should carry the character forward, but it reached {:?}",
            locomotion.position,
        );
    }

    #[test]
    fn drive_and_in_place_modes_publish_but_do_not_move() {
        let (base, params, rig) = setup();

        for mode in [RootMotion::Drive, RootMotion::InPlace] {
            let mut locomotion = Locomotion { mode, ..Default::default() };

            for i in 0..120 {
                advance(
                    &mut locomotion,
                    i as f32 / 120.0,
                    1.0,
                    &params,
                    &base,
                    &rig,
                    1.0 / 120.0,
                );
            }

            assert_eq!(
                locomotion.position,
                Vec3::ZERO,
                "{mode:?} must leave the position to its owner",
            );
            assert!(
                locomotion.root_velocity.length() > 0.0,
                "{mode:?} must still publish a velocity",
            );
        }
    }

    #[test]
    fn a_full_cycle_travels_about_one_stride() {
        // The sanity check that ties the published velocity back to the
        // gait's own geometry: two steps per cycle, so a cycle should carry
        // the body roughly the distance the feet actually cover.
        let (base, params, rig) = setup();

        let mut locomotion =
            Locomotion { mode: RootMotion::Authoritative, ..Default::default() };

        const STEPS: usize = 2000;
        for i in 0..STEPS {
            advance(
                &mut locomotion,
                i as f32 / STEPS as f32,
                1.0,
                &params,
                &base,
                &rig,
                1.0 / STEPS as f32,
            );
        }

        let travelled = -locomotion.position.z;

        // Measured: 1.04 m per cycle, so 0.52 m per step — a cycle is two
        // steps. At one cycle per second that is 1.04 m/s, against real
        // walking speeds of 1.2-1.4 m/s. Plausible, and the right order.
        //
        // Note this is NOT `stride_length * 2`. That field feeds
        // `gait::foot_offset_z`, which describes the authored INTENT, while
        // the velocity here is derived from where the rotations actually put
        // the foot — measured at 0.61 m of relative travel per stance
        // against the authored 0.45. See `GaitParams::stride_length`'s own
        // note.
        assert!(
            (0.7..=1.6).contains(&travelled),
            "a full cycle travelled {travelled} m — outside the range a human-scale \
             walk covers in two steps",
        );
    }

    #[test]
    fn six_hundred_steps_are_bit_identical_across_runs() {
        // Determinism, which this stack states as a requirement. There is no
        // RNG and no hash iteration here — the solve is a fixed sequence of
        // float operations — so this is cheap insurance rather than a live
        // worry, and it would catch an accidental dependency on iteration
        // order if one were introduced.
        let (base, params, rig) = setup();

        let run = || {
            let mut locomotion =
                Locomotion { mode: RootMotion::Authoritative, ..Default::default() };
            for i in 0..600 {
                advance(
                    &mut locomotion,
                    i as f32 / 600.0,
                    1.2,
                    &params,
                    &base,
                    &rig,
                    1.0 / 60.0,
                );
            }
            (locomotion.position, locomotion.root_velocity)
        };

        assert_eq!(run(), run());
    }

    #[test]
    fn a_character_follows_a_slope_rather_than_walking_through_it() {
        // THE gap that walking on a slope exposed. Root motion publishes
        // horizontal travel only, so on its own a character keeps its
        // starting height and sinks into rising ground: measured live at
        // 7.7 m underneath the surface after 26 m of a 0.3 grade, while
        // holding y = 0.949 throughout.
        use crate::character::anim::ground::SlopedGround;

        let ground = SlopedGround { height: 0.0, grade: 0.3 };
        let stand = 0.95;

        // Walking forward is `-Z`, and this slope rises ahead.
        for distance in [0.0_f32, 5.0, 26.0] {
            let at = Vec3::new(0.0, 0.0, -distance);
            let height = ground_following_height(at, stand, &ground).expect("ground");

            let expected = 0.3 * distance + stand;
            assert!(
                (height - expected).abs() < 1.0e-4,
                "at {distance} m along a 0.3 grade the character should stand at \
                 {expected}, got {height}",
            );
        }
    }

    #[test]
    fn flat_ground_holds_a_constant_height() {
        use crate::character::anim::ground::FlatGround;

        let ground = FlatGround { height: 0.0 };

        for distance in [0.0_f32, 5.0, 26.0] {
            let height =
                ground_following_height(Vec3::new(0.0, 0.0, -distance), 0.95, &ground)
                    .expect("ground");
            assert!((height - 0.95).abs() < 1.0e-6, "at {distance} m");
        }
    }

    #[test]
    fn no_ground_means_no_height_to_follow() {
        // Over a ledge the caller decides — fall, hover, hold the last
        // value — so this reports the absence rather than inventing one.
        struct Void;
        impl crate::character::anim::ground::GroundProbe for Void {
            fn sample(
                &self,
                _: Vec3,
            ) -> Option<crate::character::anim::ground::GroundHit> {
                None
            }
        }

        assert_eq!(ground_following_height(Vec3::ZERO, 0.95, &Void), None);
    }

    // -----------------------------------------------------------------
    // Turning
    // -----------------------------------------------------------------

    #[test]
    fn a_straight_walk_is_unchanged_by_the_turning_path() {
        // Turning must be inert when nothing turns, or every existing
        // locomotion result silently changes.
        use crate::character::anim::facing::Facing;

        let (base, params, rig) = setup();

        let mut plain = Locomotion { mode: RootMotion::Authoritative, ..Default::default() };
        let mut turning =
            Locomotion { mode: RootMotion::Authoritative, ..Default::default() };
        let mut facing = Facing::default();

        for i in 0..120 {
            let phase = i as f32 / 120.0;
            let dt = 1.0 / 120.0;

            advance(&mut plain, phase, 1.0, &params, &base, &rig, dt);
            advance_turning(
                &mut turning,
                &mut facing,
                phase,
                1.0,
                &params,
                &base,
                &rig,
                dt,
            );
        }

        assert!(
            plain.position.distance(turning.position) < 1.0e-5,
            "a character with nothing to turn toward went to {:?} instead of {:?}",
            turning.position,
            plain.position,
        );
    }

    #[test]
    fn travel_follows_the_facing_rather_than_the_world() {
        // THE point of turning. `root_velocity` is in the CHARACTER's frame,
        // so integrating it into a world position works only for a character
        // that never turns.
        use crate::character::anim::facing::Facing;

        let (base, params, rig) = setup();

        // Already facing a quarter turn — no turning during the walk, so
        // this isolates the frame conversion from the rotation itself.
        let mut facing = Facing::at(std::f32::consts::FRAC_PI_2);
        let mut locomotion =
            Locomotion { mode: RootMotion::Authoritative, ..Default::default() };

        for i in 0..120 {
            advance_turning(
                &mut locomotion,
                &mut facing,
                i as f32 / 120.0,
                1.0,
                &params,
                &base,
                &rig,
                1.0 / 120.0,
            );
        }

        // A quarter turn points the character at -X (see `facing::yaw_of`),
        // so that is where it should have gone.
        let travelled = locomotion.position;
        assert!(
            travelled.x < -0.5,
            "facing a quarter turn, the character should travel toward -X, but \
             reached {travelled:?}",
        );
        assert!(
            travelled.z.abs() < travelled.x.abs() * 0.1,
            "...and barely at all along Z, got {travelled:?}",
        );
    }

    #[test]
    fn the_cancellation_identity_survives_turning() {
        // The test most likely to expose a frame error: the body is both
        // translating AND rotating, and the stance foot still has to stand
        // still.
        //
        // The foot is tracked in the character's own frame and transformed
        // to world by the same facing the velocity was rotated by — if
        // those two disagree, the foot slides.
        use crate::character::anim::facing::Facing;

        let (base, params, rig) = setup();

        let mut facing = Facing::at(0.0);
        facing.target_yaw = 1.0;
        facing.turn_rate = 0.8;

        let mut locomotion =
            Locomotion { mode: RootMotion::Authoritative, ..Default::default() };

        let frames = 120usize;
        let dt = 1.0 / frames as f32;

        let world_foot = |phase: f32, loco: &Locomotion, facing: &Facing, bone| {
            let k = forward_kinematics_on(&walk_pose_on(phase, &params, &base, &rig), &rig);
            loco.position + facing.rotation() * (k[bone] - k[Bone::Hips])
        };

        let mut worst = 0.0f32;
        let mut previous: Option<(Bone, Vec3)> = None;

        for i in 0..frames {
            let phase = i as f32 / frames as f32;

            advance_turning(
                &mut locomotion,
                &mut facing,
                phase,
                1.0,
                &params,
                &base,
                &rig,
                dt,
            );

            let Some(bone) = stance_foot(phase, &params) else { continue };
            let now = world_foot(phase, &locomotion, &facing, bone);

            if let Some((prev_bone, prev)) = previous
                && prev_bone == bone
            {
                worst = worst.max(now.distance(prev));
            }

            previous = Some((bone, now));
        }

        // Looser than the straight-line identity (1e-3 there): the foot is
        // genuinely PIVOTING here, so its world position legitimately moves
        // — what must not happen is it sliding along the ground as well.
        // A 1 rad turn sweeps a 0.2 m-out foot about 0.2 m total, so a
        // per-frame bound well under that catches a frame error.
        assert!(
            worst < 0.01,
            "a stance foot moved {worst} m in one frame while the body turned — \
             the velocity and the foot are not in the same frame",
        );
    }

    #[test]
    fn the_returned_turn_describes_what_the_body_did() {
        // The foot locks consume this, so it has to be the body's actual
        // rotation for the frame rather than the requested one.
        use crate::character::anim::facing::Facing;

        let (base, params, rig) = setup();

        let mut facing = Facing::at(0.0);
        facing.target_yaw = 1.0;
        facing.turn_rate = 0.5;

        let mut locomotion = Locomotion::default();

        let turn = advance_turning(
            &mut locomotion,
            &mut facing,
            0.15,
            1.0,
            &params,
            &base,
            &rig,
            0.1,
        );

        // 0.5 rad/s for 0.1 s.
        assert!(
            (turn.yaw_delta - 0.05).abs() < 1.0e-5,
            "expected a 0.05 rad turn, got {}",
            turn.yaw_delta,
        );
        assert_eq!(
            turn.pivot, locomotion.position,
            "the pivot should be the body's own position",
        );
    }

    #[test]
    fn a_settled_facing_reports_no_turn() {
        use crate::character::anim::facing::Facing;

        let (base, params, rig) = setup();
        let mut facing = Facing::at(0.4);
        let mut locomotion = Locomotion::default();

        let turn = advance_turning(
            &mut locomotion,
            &mut facing,
            0.15,
            1.0,
            &params,
            &base,
            &rig,
            1.0 / 60.0,
        );

        assert_eq!(turn.yaw_delta, 0.0, "a character with nowhere to turn should not");
    }

    #[test]
    fn the_heading_advances_before_the_velocity_is_rotated_by_it() {
        // Ordering, pinned: rotate first and the character travels one frame
        // behind where it is pointing. Invisible at 60 Hz in a still frame
        // and a real lag in motion.
        use crate::character::anim::facing::Facing;

        let (base, params, rig) = setup();

        let mut facing = Facing::at(0.0);
        facing.target_yaw = 1.0;
        facing.turn_rate = 100.0; // arrives in one step

        let mut locomotion =
            Locomotion { mode: RootMotion::Authoritative, ..Default::default() };

        advance_turning(
            &mut locomotion,
            &mut facing,
            0.15,
            1.0,
            &params,
            &base,
            &rig,
            1.0 / 60.0,
        );

        // The facing arrived this frame, so the velocity must already be in
        // the NEW heading.
        let expected = Facing::at(1.0).rotation()
            * root_velocity(0.15, 1.0, &params, &base, &rig);

        assert!(
            locomotion.root_velocity.abs_diff_eq(expected, 1.0e-5),
            "the published velocity {:?} is not in the heading the character \
             reached this frame ({expected:?})",
            locomotion.root_velocity,
        );
    }

    // -----------------------------------------------------------------
    // The flight phase — a path a walk never reaches
    // -----------------------------------------------------------------

    #[test]
    fn a_run_has_moments_with_no_stance_foot() {
        // The precondition for everything below, asserted rather than
        // assumed: `stance_foot` returning `None` is a branch no walk ever
        // takes, so it was untested until there was a run.
        use crate::character::anim::gait::GaitParams;

        let params = GaitParams::running();

        let airborne = (0..400)
            .filter(|i| stance_foot(*i as f32 / 400.0, &params).is_none())
            .count();

        assert!(airborne > 0, "a run should leave the ground");
        assert!(
            airborne < 200,
            "...but not for half the cycle — that is a jump, not a run",
        );
    }

    #[test]
    fn the_body_coasts_through_a_flight_phase_rather_than_stopping() {
        // THE flight-phase bug. With no contact to derive a velocity from,
        // the walk-only code returned ZERO — a dead stop twice per cycle.
        // At a 0.4 duty factor flight is 20% of the cycle, so at
        // 1.5 strides/s the body would stall for 67 ms, twice a second.
        //
        // A body in flight is a projectile: it keeps the velocity it left
        // the ground with.
        use crate::character::anim::gait::GaitParams;

        let (base, _, rig) = setup();
        let params = GaitParams::running();

        let mut airborne_speeds = Vec::new();

        for i in 0..400 {
            let phase = i as f32 / 400.0;
            if stance_foot(phase, &params).is_some() {
                continue;
            }

            airborne_speeds
                .push(root_velocity(phase, 1.5, &params, &base, &rig).length());
        }

        assert!(!airborne_speeds.is_empty(), "test setup: expected a flight phase");

        let slowest =
            airborne_speeds.iter().copied().fold(f32::INFINITY, f32::min);

        assert!(
            slowest > 0.1,
            "the body stalled to {slowest} m/s mid-flight — it should coast at the \
             speed it took off with",
        );
    }

    #[test]
    fn the_velocity_is_continuous_across_a_flight_phase() {
        // Coasting is only right if it MATCHES what the body had at
        // toe-off. A coast at the wrong speed is a different lurch, not a
        // fix.
        //
        // Contact HANDOVERS are excluded, the same way the walk's own
        // continuity test excludes them: when the reference moves from one
        // foot to the other the velocity legitimately changes, because the
        // two feet are at different points in their own stance curves. What
        // must not change is the velocity while tracking a single foot —
        // and, here, across the flight phase between them.
        use crate::character::anim::gait::GaitParams;

        let (base, _, rig) = setup();
        let params = GaitParams::running();

        let mut previous = root_velocity(0.0, 1.5, &params, &base, &rig);
        let mut previous_foot = stance_foot(0.0, &params);
        let mut worst = 0.0f32;

        for i in 1..=800 {
            let phase = i as f32 / 800.0;
            let now = root_velocity(phase, 1.5, &params, &base, &rig);
            let foot = stance_foot(phase, &params);

            // A handover is a change of REFERENCE between two grounded
            // feet; a flight phase (either side `None`) is exactly what
            // this test is here to check, so it is not excluded.
            let handover = matches!((previous_foot, foot), (Some(a), Some(b)) if a != b);

            if !handover {
                worst = worst.max((now - previous).length());
            }

            previous = now;
            previous_foot = foot;
        }

        assert!(
            worst < 0.5,
            "the published velocity jumped by {worst} m/s between adjacent samples \
             — the coast does not match the speed at toe-off",
        );
    }

    #[test]
    fn a_running_character_actually_travels() {
        use crate::character::anim::gait::GaitParams;

        let (base, _, rig) = setup();
        let params = GaitParams::running();

        let mut locomotion =
            Locomotion { mode: RootMotion::Authoritative, ..Default::default() };

        const STEPS: usize = 2000;
        for i in 0..STEPS {
            advance(
                &mut locomotion,
                i as f32 / STEPS as f32,
                1.5,
                &params,
                &base,
                &rig,
                1.0 / STEPS as f32,
            );
        }

        let travelled = -locomotion.position.z;
        assert!(
            travelled > 1.0,
            "a full running cycle covered only {travelled} m",
        );
    }

    #[test]
    fn a_degenerate_gait_cannot_publish_an_absurd_velocity() {
        // `leg_phase` clamps a duty factor to a 0.01 minimum rather than
        // rejecting it, so "no contact at all" is really a 1% stance
        // sliver — and differentiating across a sliver where the foot
        // effectively teleports between stance and swing measured
        // **156 m/s**, enough to fling a character across a level in a
        // frame.
        //
        // The ceiling is expressed against the leg's own reach and the
        // cadence, so it scales with the rig: a foot cannot travel more
        // than a few leg-lengths per stride.
        use crate::character::anim::gait::GaitParams;

        let (base, _, rig) = setup();

        for duty in [0.0_f32, 0.01, 0.99, 1.0, -1.0, f32::NAN] {
            let params = GaitParams { duty_factor: duty, ..GaitParams::running() };

            for i in 0..50 {
                let phase = i as f32 / 50.0;
                let velocity = root_velocity(phase, 1.5, &params, &base, &rig);

                assert!(
                    velocity.is_finite(),
                    "duty {duty} at phase {phase} produced {velocity:?}",
                );
                assert!(
                    velocity.length() < 10.0,
                    "duty {duty} at phase {phase} published {} m/s, which no leg \
                     can produce",
                    velocity.length(),
                );
            }
        }
    }

    #[test]
    fn a_zero_timestep_changes_nothing() {
        let (base, params, rig) = setup();

        let mut locomotion =
            Locomotion { mode: RootMotion::Authoritative, ..Default::default() };
        advance(&mut locomotion, 0.15, 1.0, &params, &base, &rig, 0.0);

        assert_eq!(locomotion.position, Vec3::ZERO);
        assert!(locomotion.root_velocity.length() > 0.0, "but it still publishes");
    }

    #[test]
    fn a_walking_body_sways_over_its_stance_feet_but_never_past_them() {
        // Winter §11.3.1: in steady walking the centre of mass weaves toward
        // each stance foot and passes just medial of its inside border,
        // never over it. The sway turns both legs about their ankles, so
        // the feet stay exactly where the walk put them — which is also
        // what keeps it out of root motion.
        use crate::character::anim::anthropometry::centre_of_mass;
        use crate::character::anim::foot::Sole;
        use crate::character::anim::gait::leg_phase;
        use crate::character::anim::phase::{GaitPhase, PhaseLayer, WalkSway};
        let (stood, _, rig) = real_walk();
        let left = rig.left();
        let soles = [Sole::of(&rig, Bone::LeftFoot), Sole::of(&rig, Bone::RightFoot)];
        let sole = |pose: &LocalPose, leg: usize| soles[leg].points(pose, &rig).map(|p| p + pose.root_translation);
        // `puppet_base`'s foot mesh: its inside border is 3.8 cm medial of
        // the sole's centreline (measured from the vertices skinned to
        // foot_l / ball_l, 11 cm wide).
        const INNER_BORDER: f32 = 0.038;
        let layer = PhaseLayer { walk_sway: Some(WalkSway { gain: 1.0 }), ..PhaseLayer::none() };

        for speed in [0.7, 1.2, 1.6] {
            let params = GaitParams::walking_on(speed, &rig);
            let distance = distance_per_cycle(&params, &stood, &rig);
            let (mut toward, mut margin, mut feet_moved) = (0.0f32, f32::MAX, 0.0f32);
            let (mut swing_across, mut swing_lowered) = (0.0f32, 0.0f32);
            let (mut double_moved, mut clearance) = (0.0f32, [f32::MAX; 2]);
            let ground = [0, 1].map(|leg| sole(&stood, leg).iter().map(|p| p.y).fold(f32::MAX, f32::min));
            for i in 0..64 {
                let cycle = i as f32 / 64.0;
                let walked = walk_pose_on(cycle, &params, &stood, &rig);
                let clock = GaitPhase {
                    gait: cycle * std::f32::consts::TAU,
                    speed,
                    base_frequency_hz: 0.0,
                    speed_coefficient: 1.0 / distance,
                    ..Default::default()
                };
                let mut swayed = walked;
                layer.apply_on(&clock, &mut swayed, &rig);
                let stance = [0.0, 0.5].map(|shift| leg_phase(cycle + shift, params.duty_factor).is_stance());
                let single = stance[0] != stance[1];
                for leg in 0..2 {
                    let (a, b) = (sole(&walked, leg), sole(&swayed, leg));
                    if !stance[leg] {
                        let low = |c: &[Vec3; 3]| c.iter().map(|p| p.y).fold(f32::MAX, f32::min);
                        clearance[0] = clearance[0].min(low(&a) - ground[leg]);
                        clearance[1] = clearance[1].min(low(&b) - ground[leg]);
                    }
                    for k in 0..3 {
                        let moved = b[k] - a[k];
                        if stance[leg] && !single {
                            double_moved = double_moved.max(moved.length());
                        } else if stance[leg] {
                            feet_moved = feet_moved.max(moved.length());
                        } else {
                            // A swinging foot keeps its path over the
                            // ground; its height is the pelvis's to set.
                            swing_across = swing_across.max(Vec3::new(moved.x, 0.0, moved.z).length());
                            swing_lowered = swing_lowered.max(-moved.y);
                        }
                    }
                }
                if stance[0] == stance[1] {
                    continue;
                }
                // Single support: signed toward the stance foot.
                let (leg, side) = if stance[0] { (0, 1.0) } else { (1, -1.0) };
                let pelvis = (swayed.root_translation - walked.root_translation).dot(left) * side;
                toward = toward.max(pelvis);
                assert!(pelvis > -1.0e-3, "at {speed} m/s, cycle {cycle}: the pelvis leans {pelvis} m away from the stance foot");
                let com = (swayed.root_translation + centre_of_mass(&swayed, &rig)).dot(left) * side;
                let centreline = sole(&swayed, leg).iter().map(|p| p.dot(left)).sum::<f32>() / 3.0 * side;
                margin = margin.min(centreline - INNER_BORDER - com);
            }
            // Measured 0.02 mm in single support; 0.57-0.67 mm in double
            // support, where the trailing leg is near full extension and
            // cannot give the last fraction of a millimetre.
            assert!(feet_moved < 1.0e-4, "at {speed} m/s the sway moved the planted foot {:.2} mm", feet_moved * 1e3);
            assert!(double_moved < 1.0e-3, "at {speed} m/s the sway moved a planted foot {:.2} mm in double support", double_moved * 1e3);
            assert!(swing_across < 1.0e-4, "at {speed} m/s the sway moved a swinging foot {:.2} mm across the ground", swing_across * 1e3);
            assert!(swing_lowered < 1.0e-4, "at {speed} m/s the sway lowered a swinging foot {:.2} mm", swing_lowered * 1e3);
            assert!(clearance[1] > clearance[0] - 1.0e-4, "at {speed} m/s the sway cost the swing {:.2} mm of clearance", (clearance[0] - clearance[1]) * 1e3);
            assert!(toward > 0.01, "at {speed} m/s the pelvis should sway over 1 cm toward the stance foot, got {toward}");
            // Just medial: inside the border, and within a few centimetres of it.
            assert!(
                (0.0..0.03).contains(&margin),
                "at {speed} m/s the centre of mass passes {:.1} mm medial of the stance foot's inside border",
                margin * 1e3
            );
        }
    }

    #[test]
    fn a_walking_pelvis_drops_on_the_swing_side_with_the_trunk_upright() {
        // Winter §7.4.5 on the real rig: the hip socket over the swinging
        // leg drops in early stance (lowest ~17 % after the stance heel
        // contact), about the stance hip, while the trunk stays upright.
        use crate::character::anim::phase::{GaitPhase, PhaseLayer, WalkSway};
        use crate::character::anim::rig::{accumulate_world_rotations, offset_from};
        let (stood, _, rig) = real_walk();
        let layer = PhaseLayer { walk_sway: Some(WalkSway { gain: 1.0 }), ..PhaseLayer::none() };
        let params = GaitParams::walking_on(1.2, &rig);
        let distance = distance_per_cycle(&params, &stood, &rig);
        let posed = |cycle: f32| {
            let mut pose = walk_pose_on(cycle, &params, &stood, &rig);
            let clock = GaitPhase {
                gait: cycle * std::f32::consts::TAU,
                speed: 1.2,
                base_frequency_hz: 0.0,
                speed_coefficient: 1.0 / distance,
                ..Default::default()
            };
            layer.apply_on(&clock, &mut pose, &rig);
            pose
        };
        // Left socket's height over the right's, as an angle across them.
        let roll = |pose: &LocalPose| {
            let across = offset_from(pose, &rig, Bone::Hips, Bone::LeftUpLeg) - offset_from(pose, &rig, Bone::Hips, Bone::RightUpLeg);
            across.y.atan2(across.dot(rig.left()))
        };
        let bare = |cycle: f32| roll(&walk_pose_on(cycle, &params, &stood, &rig));
        let (mut lowest_at, mut lowest) = (0.0, 0.0f32);
        let mut trunk = 0.0f32;
        for i in 0..100 {
            let cycle = i as f32 / 200.0; // left stance, right swinging
            let pose = posed(cycle);
            let added = roll(&pose) - bare(cycle);
            if added > lowest {
                (lowest_at, lowest) = (cycle, added);
            }
            let (walked, rolled) = (
                accumulate_world_rotations(&walk_pose_on(cycle, &params, &stood, &rig), &rig),
                accumulate_world_rotations(&pose, &rig),
            );
            trunk = trunk.max(walked[Bone::Spine2].angle_between(rolled[Bone::Spine2]));
        }
        assert!((0.14..0.22).contains(&lowest_at), "the right (swing) side is lowest at {lowest_at:.3} of the stride");
        assert!(
            (3.0..4.5).contains(&lowest.to_degrees()),
            "the right (swing) side drops {:.2}° against the left",
            lowest.to_degrees()
        );
        assert!(trunk < 0.01, "the trunk tipped {:.2}° with the pelvis", trunk.to_degrees());
    }

    #[test]
    fn the_pelvis_turns_with_the_stepping_leg_and_the_chest_against_it() {
        // Each side of the pelvis is furthest forward at its own heel
        // contact (the stance hip rotators brake it just after, Winter
        // §7.4.5 H1-T); the chest turns the other way with the arms.
        use crate::character::anim::phase::{GaitPhase, PhaseLayer, PELVIC_ROTATION};
        use crate::character::anim::rig::offset_from;
        let (stood, _, rig) = real_walk();
        let (left, fwd) = (rig.left(), rig.forward());
        let layer = PhaseLayer::locomotion();
        let params = GaitParams::walking_on(1.2, &rig);
        let distance = distance_per_cycle(&params, &stood, &rig);
        // Positive: the rig's left side ahead.
        let yaw = |pose: &LocalPose, l: Bone, r: Bone| {
            let across = offset_from(pose, &rig, Bone::Hips, l) - offset_from(pose, &rig, Bone::Hips, r);
            across.dot(fwd).atan2(across.dot(left))
        };
        let samples: Vec<(f32, f32, f32)> = (0..100)
            .map(|i| {
                let cycle = i as f32 / 100.0;
                let mut pose = walk_pose_on(cycle, &params, &stood, &rig);
                let bare = yaw(&pose, Bone::LeftUpLeg, Bone::RightUpLeg);
                let clock = GaitPhase {
                    gait: cycle * std::f32::consts::TAU,
                    speed: 1.2,
                    base_frequency_hz: 0.0,
                    speed_coefficient: 1.0 / distance,
                    ..Default::default()
                };
                layer.apply_on(&clock, &mut pose, &rig);
                (cycle, yaw(&pose, Bone::LeftUpLeg, Bone::RightUpLeg) - bare, yaw(&pose, Bone::LeftArm, Bone::RightArm))
            })
            .collect();
        let extreme = |pick: fn(&(f32, f32, f32)) -> f32| {
            let most = samples.iter().max_by(|a, b| pick(a).total_cmp(&pick(b))).unwrap();
            let least = samples.iter().min_by(|a, b| pick(a).total_cmp(&pick(b))).unwrap();
            ((most.0, pick(most)), (least.0, pick(least)))
        };
        let near = |cycle: f32, at: f32| {
            let d = (cycle - at).rem_euclid(1.0);
            d.min(1.0 - d) < 0.06
        };
        let ((left_ahead_at, left_ahead), (right_ahead_at, right_ahead)) = extreme(|s| s.1);
        assert!(near(left_ahead_at, 0.0) && near(right_ahead_at, 0.5), "pelvis extremes at {left_ahead_at} / {right_ahead_at}");
        for turned in [left_ahead, -right_ahead] {
            assert!((turned - PELVIC_ROTATION).abs() < 0.01, "the pelvis turns {:.2}°", turned.to_degrees());
        }
        // The chest: right shoulder ahead as the left heel lands.
        let ((chest_left_at, _), (chest_right_at, _)) = extreme(|s| s.2);
        assert!(near(chest_right_at, 0.0) && near(chest_left_at, 0.5), "chest extremes at {chest_right_at} / {chest_left_at}");
    }

    #[test]
    fn the_rendered_chest_turns_against_the_pelvis_on_time() {
        // The chest's twist is timed by the gait, like the arm swing. A
        // spring slow enough to feel weighty low-passes it: on the spine's
        // 0.16 s the rendered chest kept 0.36 of its turn and peaked 107°
        // late — live, it swung ±1.8° and was uncorrelated with the pelvis.
        use crate::character::anim::dho::{default_springs, DhoState};
        use crate::character::anim::phase::{GaitPhase, PhaseLayer};
        use crate::character::anim::rig::offset_from;
        let (stood, _, rig) = real_walk();
        let (left, fwd) = (rig.left(), rig.forward());
        let layer = PhaseLayer::locomotion();
        let params = GaitParams::walking_on(1.2, &rig);
        let distance = distance_per_cycle(&params, &stood, &rig);
        let springs = default_springs();
        let mut clock =
            GaitPhase { speed: 1.2, base_frequency_hz: 0.0, speed_coefficient: 1.0 / distance, ..Default::default() };
        let target_at = |clock: &GaitPhase| {
            let mut pose = walk_pose_on(clock.gait / std::f32::consts::TAU, &params, &stood, &rig);
            layer.apply_on(clock, &mut pose, &rig);
            pose
        };
        // Positive: the left shoulder ahead.
        let chest = |pose: &LocalPose| {
            let across = offset_from(pose, &rig, Bone::Hips, Bone::LeftArm) - offset_from(pose, &rig, Bone::Hips, Bone::RightArm);
            across.dot(fwd).atan2(across.dot(left))
        };
        let mut dho = DhoState::settled_on(&target_at(&clock));
        let dt = 1.0 / 60.0;
        let (mut rendered, mut target) = (Vec::new(), Vec::new());
        for frame in 0..300 {
            clock.advance(dt);
            let goal = target_at(&clock);
            dho.advance(&goal, &springs, dt);
            if frame >= 120 {
                let cycle = clock.gait / std::f32::consts::TAU;
                rendered.push((cycle, chest(&dho.pose(goal.root_translation))));
                target.push((cycle, chest(&goal)));
            }
        }
        let span = |s: &[(f32, f32)]| {
            s.iter().map(|p| p.1).fold(f32::MIN, f32::max) - s.iter().map(|p| p.1).fold(f32::MAX, f32::min)
        };
        assert!(span(&rendered) > 0.8 * span(&target), "the rendered chest turns {:.2}° of the target's {:.2}°",
            span(&rendered).to_degrees(), span(&target).to_degrees());
        // The right shoulder is furthest ahead within a tenth of a stride
        // after the left heel lands.
        let (at, _) = rendered.iter().copied().min_by(|a, b| a.1.total_cmp(&b.1)).unwrap();
        let late = (at - 0.0).rem_euclid(1.0);
        assert!(late < 0.1 || late > 0.97, "the rendered chest's right-shoulder peak is {late:.3} of a stride after the left heel");
    }

    #[test]
    fn the_rendered_trunk_stays_upright_while_the_pelvis_rolls() {
        // The target counter-rolls the trunk against the pelvis; on screen
        // that holds only if the springs deliver both together. With the
        // first spine bone on the trunk's 0.16 s against the hips' 0.015 s,
        // the sprung trunk rolled with the pelvis: 8.1° peak to peak live.
        use crate::character::anim::dho::{default_springs, DhoState};
        use crate::character::anim::phase::{GaitPhase, PhaseLayer, WalkSway};
        use crate::character::anim::rig::accumulate_world_rotations;
        let (stood, _, rig) = real_walk();
        let layer = PhaseLayer { walk_sway: Some(WalkSway { gain: 1.0 }), ..PhaseLayer::none() };
        let params = GaitParams::walking_on(1.2, &rig);
        let distance = distance_per_cycle(&params, &stood, &rig);
        let springs = default_springs();
        let mut clock =
            GaitPhase { speed: 1.2, base_frequency_hz: 0.0, speed_coefficient: 1.0 / distance, ..Default::default() };
        let target_at = |clock: &GaitPhase| {
            let mut pose = walk_pose_on(clock.gait / std::f32::consts::TAU, &params, &stood, &rig);
            layer.apply_on(clock, &mut pose, &rig);
            pose
        };
        let mut dho = DhoState::settled_on(&target_at(&clock));
        let dt = 1.0 / 60.0;
        // Lateral lean of the chest's up axis, signed across the rig's left.
        let lean = |pose: &LocalPose| {
            let up = accumulate_world_rotations(pose, &rig)[Bone::Spine2] * (accumulate_world_rotations(&LocalPose::REST, &rig)[Bone::Spine2].inverse() * Vec3::Y);
            up.dot(rig.left()).atan2(up.y)
        };
        let (mut rendered, mut target) = ((f32::MAX, f32::MIN), (f32::MAX, f32::MIN));
        for frame in 0..240 {
            clock.advance(dt);
            let goal = target_at(&clock);
            dho.advance(&goal, &springs, dt);
            if frame >= 60 {
                let (r, t) = (lean(&dho.pose(goal.root_translation)), lean(&goal));
                rendered = (rendered.0.min(r), rendered.1.max(r));
                target = (target.0.min(t), target.1.max(t));
            }
        }
        let span = |(lo, hi): (f32, f32)| (hi - lo).to_degrees();
        assert!(span(target) < 1.0, "the target trunk leans {:.2}° peak to peak", span(target));
        assert!(span(rendered) < 1.5, "the rendered trunk leans {:.2}° peak to peak", span(rendered));
    }
}
