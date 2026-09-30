//! A standing stance with softly bent knees — the pose everything else
//! should build on, and the thing that makes foot IK solvable at all.
//!
//! # Why a bind pose cannot be stood on
//!
//! A rig is bound straight-legged: the hip socket sits at exactly
//! femur + shin + ankle above the sole. Measured on this project's own two
//! rigs, both agree to four decimals — `puppet_base.gltf` has its hip
//! socket at `0.9712` and a straight-leg reach of `0.9712`, and the
//! synthetic T-pose table is `0.940` and `0.940`. That is not a defect in
//! either; it is what a T-pose bind *is*.
//!
//! But it means the knee sits at exactly 180 degrees, which is a
//! singularity for inverse kinematics:
//!
//! - There is **zero horizontal reach budget**. A foot cannot move even a
//!   centimetre sideways without the leg needing to be longer than it is.
//! - The knee has no bend *direction*. At exactly straight, "bend forward"
//!   and "bend backward" are equally valid, so a solver has nothing to
//!   disambiguate with and will pick arbitrarily — or flip between them
//!   frame to frame.
//!
//! This is why earlier attempts at foot IK on this rig measured up to
//! **2.18x leg overstretch**: they were not mis-tuned, the pose they
//! started from had no solution space.
//!
//! # The fix is authored data, not rig geometry
//!
//! A real person standing relaxed carries 5-15 degrees of knee flexion.
//! Authoring that as a pose — rather than editing the rig's bone lengths —
//! is better in three ways:
//!
//! - **It works on any rig.** The stance is expressed as rotations, and
//!   rotations are rig-independent (see [`super::retarget`]). A taller or
//!   shorter character gets a correct stance from the same data.
//! - **It leaves the bind pose clean.** `t_pose_offset` stays the pure,
//!   straight-chain reference that the retargeting math and every bone
//!   length assumption depend on.
//! - **It is tunable without a recompile**, because it is a pose like any
//!   other and can live in a `.pose.ron` file.
//!
//! # The geometry
//!
//! Bending only the knee would swing the foot backward and lift the sole
//! off the ground. Three rotations are needed, and they must sum correctly:
//!
//! ```text
//! hip:   +flex/2    (thigh leans forward)
//! knee:  -flex      (shin folds back under it)
//! ankle: +flex/2    (foot re-levels against the ground)
//! ```
//!
//! With the hip and ankle each taking half the knee's angle, the shin ends
//! up back under the hip and the foot stays flat.
//!
//! Note what this symmetric split costs: the hip's `+flex/2` cancels half
//! the knee's `-flex`, so each segment ends up folded by `flex/2` rather
//! than `flex`, and the leg shortens by `L * (1 - cos(flex/2))` — about
//! half what a bare knee bend would give. That is the price of keeping the
//! shin vertical and the foot flat, and it is worth paying.
//!
//! The shortening is what buys reach, and it is deliberately small: 9
//! degrees of flex costs about 3 mm of height and yields roughly 0.055 m of
//! horizontal foot placement. Budget grows with the sine of the fold while
//! the crouch grows with its cosine, so the first few degrees are nearly
//! free and later ones are not — reaching 0.15 m would take past 20
//! degrees, which reads as a squat. Prefer a little foot sliding over a
//! permanently crouching character.

use bevy::math::{Quat, Vec3};

use super::rig::LocalPose;
use crate::character::skeleton::Bone;

/// The axis a knee bends about.
///
/// `+X` is the character's right (see [`Bone::t_pose_offset`]'s own
/// convention), and a positive rotation about it swings a bone **forward**
/// — toward `-Z`, the direction the rig faces. Both legs share the axis: a
/// knee is a hinge, and both knees hinge the same way.
///
/// # The sign, derived rather than recalled
///
/// This comment previously claimed the opposite, and the walk cycle built on
/// it ran backward. The derivation, so it can be checked rather than
/// trusted: a leg hangs along `-Y`, and
///
/// ```text
/// Rx(t) * (0, -L, 0) = (0, -L*cos(t), -L*sin(t))
/// ```
///
/// so a positive `t` drives `z` negative. `-Z` is forward. Measured the same
/// way on the real rig: `+0.3` rad moves `LeftFoot` from `z = +0.002` to
/// `z = -0.191`.
///
/// Because a knee only flexes one way, knee rotations here are NEGATIVE —
/// see [`stance_on`], which applies `-knee_flex` at the knee.
pub const KNEE_AXIS: Vec3 = Vec3::X;

/// How far the knees bend in the default standing stance, radians.
///
/// About 9 degrees: inside the 5-15 degree range a relaxed person actually
/// holds, and enough to buy roughly 0.24 m of horizontal foot placement on
/// a human-scale rig. Larger values buy more IK headroom at the cost of
/// looking like a crouch — the "avoid the dinosaur" warning from the
/// foot-locking literature, and the reason this is not simply maximised.
pub const DEFAULT_KNEE_FLEX: f32 = 0.16;

/// Builds a standing stance with both knees softly bent.
///
/// Composed **onto** `base`, so an authored upper-body pose keeps its arms
/// and spine and gains legs it can stand on.
///
/// `knee_flex` is the knee's own bend in radians; the hip and ankle each
/// take half of it in the opposite direction so the shin stays vertical and
/// the sole stays flat. See the module doc for why all three are needed.
pub fn stance_on(base: &LocalPose, knee_flex: f32) -> LocalPose {
    stance_on_rig(base, knee_flex, &super::rig::RigGeometry::default())
}

/// [`stance_on`], bending the knees the way THIS rig's own geometry says is
/// forward.
///
/// # Why the rig is needed to bend a knee
///
/// [`KNEE_AXIS`] is a constant, and the sign that swings a bone forward
/// about it is not: it depends on which way the rig faces, and rigs
/// disagree. The synthetic T-pose faces `-Z`; `puppet_base.gltf` faces
/// `+Z`. So the same rotation that bends a knee correctly on one bends it
/// BACKWARD on the other.
///
/// That is exactly what shipped. Measured on `puppet_base` before this
/// existed, as the knee's offset from the hip-to-ankle line along the rig's
/// own forward: `LocalPose::REST` sat at `+0.0169` (correct) while
/// `stance(REST)` sat at `-0.0098`, and the walk cycle composed on top
/// amplified it to `-0.19`. The character walked on backward-bending
/// knees.
///
/// It survived thirty-odd leg tests because every one of them measured the
/// UNSIGNED angle between thigh and shin, which is identical whichever way
/// the knee folds. See [`RigGeometry::knee_forward_offset`], which is the
/// signed measurement, and
/// `the_stance_bends_both_knees_forward_on_every_rig`, which asserts it.
pub fn stance_on_rig(
    base: &LocalPose,
    knee_flex: f32,
    rig: &super::rig::RigGeometry,
) -> LocalPose {
    let mut pose = *base;

    if knee_flex == 0.0 {
        return pose;
    }

    // Which way a positive rotation about `KNEE_AXIS` actually swings a
    // bone on this rig. `+1` reproduces exactly what this function did
    // before the rig was a parameter.
    let flex = knee_flex * facing_sign(rig);
    let half = flex * 0.5;
    let ankles = |pose: &LocalPose| {
        use super::rig::offset_from;
        0.5 * (offset_from(pose, rig, Bone::Hips, Bone::LeftFoot).y
            + offset_from(pose, rig, Bone::Hips, Bone::RightFoot).y)
    };
    let before = ankles(&pose);

    for (hip, knee, ankle) in [
        (Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot),
        (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot),
    ] {
        compose(&mut pose, hip, Quat::from_axis_angle(KNEE_AXIS, half));
        compose(&mut pose, knee, Quat::from_axis_angle(KNEE_AXIS, -flex));
        compose(&mut pose, ankle, Quat::from_axis_angle(KNEE_AXIS, half));
    }

    // Bent knees make shorter legs: the hips come down by what the ankles
    // rose, so the soles stay on the floor. Left standing at full height,
    // `puppet_base`'s feet floated 6.9 mm in the stance, and its legs,
    // authored at full extension, had nothing to reach them with: the foot
    // IK pitched each foot 4.5° toe-down instead. (Hidden until the live
    // rig geometry put the hips where the renderer does; it had them 9 mm
    // low, which read as slack.)
    pose.root_translation.y -= ankles(&pose) - before;

    pose
}

/// `+1` if this rig faces the way the poses here were authored, `-1` if it
/// faces the other way.
///
/// The one number every leg-posing path needs to stay correct on a rig that
/// was not exported facing [`AUTHORED_FORWARD`]. Derived from the rig's own
/// measured geometry, never assumed — see [`RigGeometry::forward`].
pub fn facing_sign(rig: &super::rig::RigGeometry) -> f32 {
    let forward = rig.forward();
    if forward == Vec3::ZERO {
        // Nothing to measure. Keep the authored convention rather than
        // picking a direction from noise.
        return 1.0;
    }

    let agreement = forward.dot(AUTHORED_FORWARD);
    if agreement.abs() < 1.0e-3 { 1.0 } else { agreement.signum() }
}

/// The facing every pose in this crate is authored against.
///
/// `-Z`, the synthetic T-pose's own convention — see
/// [`Bone::t_pose_offset`]. A rig facing the other way needs its leg angles
/// negated; [`facing_sign`] reports which.
pub const AUTHORED_FORWARD: Vec3 = Vec3::NEG_Z;

/// Shifts the pelvis by `shift` (horizontal, in the rig's frame) over feet
/// that stay exactly where they were, flat on the ground.
///
/// How a standing body sways (Winter §11.2.1): the pelvis carries the trunk
/// over the feet, each leg turning as a whole about its ankle, the trunk
/// upright. Each thigh takes the world turn that re-aims its hip-to-ankle
/// line at the ankle's old place, and each foot the opposite turn, so it
/// keeps its attitude on the ground.
///
/// A leg turned whole keeps its length, so the pelvis moves on a circle
/// about the ankles — an inverted pendulum — and rises or falls with the
/// shift. Leaving it level lifted the feet instead: the relaxed stance
/// stands with its hips ~5 cm ahead of its ankles, so a 4 cm forward lean
/// carried them 3.3 mm off the floor. The vertical is the mean the two legs
/// ask for; they differ only by the stance's small side-to-side asymmetry.
pub fn sway_over_feet(pose: &mut LocalPose, rig: &super::rig::RigGeometry, shift: Vec3) {
    sway_over_loaded_feet(pose, rig, shift, [0.5, 0.5]);
}

/// [`sway_over_feet`] for a body resting on its feet unequally: the pelvis
/// takes the vertical that keeps the loaded feet down, each leg's need
/// weighted by `loads` (left, right; normalized here).
///
/// A walk rests on one foot while the other swings: averaged in, the
/// swinging leg's need moved the planted foot 1.3 mm.
pub fn sway_over_loaded_feet(pose: &mut LocalPose, rig: &super::rig::RigGeometry, shift: Vec3, loads: [f32; 2]) {
    move_pelvis_over_feet(pose, rig, shift, Quat::IDENTITY, loads);
}

/// Carries the pelvis `shift` sideways/forward and turns it by `turn` (a
/// world rotation: the walk's roll about the rig's forward and yaw about
/// up), over feet that stay exactly where they are, with the trunk held
/// where it was.
///
/// - **Height.** The pelvis takes the vertical that keeps the loaded legs
///   their length, each leg's need weighted by `loads` (left, right;
///   normalized here). Averaged equally in a walk, the swinging leg's need
///   moved the planted foot 1.2 mm in double support and put the swinging
///   toe 0.6 mm into the floor.
/// - **Turn.** About the hip carrying the body, as a walking pelvis drops
///   on its swing side (Winter §7.4.5: the stance abductors brake the drop,
///   then lift it back) and turns over the stance hip. Rolled about its own
///   centre, the stance socket would rise, and this rig's legs have no
///   length to spare for it.
/// - **Legs.** Each is re-solved once onto its old ankle (`keep_ankle`), the
///   foot turned back to its old attitude.
pub fn move_pelvis_over_feet(
    pose: &mut LocalPose,
    rig: &super::rig::RigGeometry,
    shift: Vec3,
    turn: Quat,
    loads: [f32; 2],
) {
    move_pelvis_and_feet(pose, rig, shift, turn, loads, [Vec3::ZERO; 2], 0.0, [Vec3::ZERO; 2], [0.0; 2], |needed, _| needed);
}

/// How far a trailing foot's heel may rise about its ball, radians, when
/// [`move_pelvis_and_feet`] is let use it. Winter's heel is past 45 degrees
/// by toe-off (Table A.3(a)); a standing body stepping out stays well short.
pub const MAX_HEEL_RISE: f32 = 0.6;

/// How far the pelvis drops for a long stance with the feet flat before a
/// trailing heel rises instead, metres. The walking pelvis's own
/// excursion is ~4-5 cm; held flat, the rear foot of a 0.4 m stumble step
/// asked 14 cm once the weight was over the front foot.
pub const DROP_BEFORE_HEEL_RISE: f32 = 0.04;

/// [`move_pelvis_over_feet`], with the feet also moved: each by `feet`
/// (left, right; the pose's frame, lift included) from where it stands.
///
/// One solve, so the pelvis's height accounts for where each loaded foot is
/// going, and each ankle is placed once under the moved pelvis. Placing a
/// stepped foot first, under a pelvis not yet carried over it, left it out
/// of reach (short, in the air) and the pelvis then kept that.
///
/// The pelvis height: `aims` (each foot's displacement to judge the need
/// by: a swinging foot's point ahead on its arc) give the height `settle`
/// is offered as a target; the ceiling is each loaded leg's need for where
/// its foot is now, plus its `slack`. `settle(target, ceiling)` returns the
/// drop to pose, held at or under the ceiling.
#[allow(clippy::too_many_arguments)]
pub fn move_pelvis_and_feet(
    pose: &mut LocalPose,
    rig: &super::rig::RigGeometry,
    shift: Vec3,
    turn: Quat,
    loads: [f32; 2],
    feet: [Vec3; 2],
    rise: f32,
    aims: [Vec3; 2],
    slack: [f32; 2],
    settle: impl FnOnce(f32, f32) -> f32,
) {
    use super::rig::{delta_after_world_turn, offset_from};
    let shift = Vec3::new(shift.x, 0.0, shift.z);
    let turned = 1.0 - turn.dot(Quat::IDENTITY).abs() > 1.0e-12;
    if shift.length_squared() < 1.0e-12 && !turned && feet == [Vec3::ZERO; 2] {
        return;
    }
    let total = loads[0].max(0.0) + loads[1].max(0.0);
    let loads = if total > 1.0e-6 { loads.map(|l| l.max(0.0) / total) } else { [0.5, 0.5] };
    // Each leg: its bones, its hip socket and the hip-to-ankle line.
    let legs = [(Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot), (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot)]
        .map(|(socket, knee, ankle)| {
            let hip = offset_from(pose, rig, Bone::Hips, socket);
            ([socket, knee, ankle], hip, offset_from(pose, rig, Bone::Hips, ankle) - hip)
        });
    // How far the pelvis must drop (negative: rise) for each leg to keep
    // its length under the shift: |leg − shift − v·Y| = |leg|. The turn
    // pivots on the loaded socket, so it asks nothing more of the loaded
    // leg.
    //
    // The most any loaded leg asks, not their load-weighted mean: sat
    // higher than that, a loaded leg cannot reach its foot. With the feet
    // a 0.4 m step apart the mean left the rear leg short, and its planted
    // foot lifted 42 mm. The legs asking less bend their knees for it.
    // `drop` is the root's rise, so the most asked is the least: taking
    // the greatest still lifted the rear foot 37 mm as the COM went over
    // the stepped one.
    //
    // A foot trailing its hip may instead rise on its toes, up to `rise`
    // (`MAX_HEEL_RISE`), once flat it would ask more than
    // `DROP_BEFORE_HEEL_RISE`: a lunge's rear foot, not a squat. It rolls
    // rigidly about its sole's tip, as `foot::Sole` models a foot and as
    // Winter's pre-swing does (the metatarsal marker climbs, the toe marker
    // stays down): about the ball, the rigid tip went into the floor.
    let heels = legs.map(|([_, _, ankle], hip, leg)| {
        if rise <= 0.0 {
            return (Vec3::ZERO, Vec3::ZERO);
        }
        let tip = super::foot::Sole::of(rig, ankle).points(pose, rig)[2] - (hip + leg);
        let forward = Vec3::new(tip.x, 0.0, tip.z).normalize_or_zero();
        (tip, Vec3::Y.cross(forward))
    });
    let trailing = [0, 1].map(|i| {
        let (_, _, leg) = legs[i];
        rise > 0.0 && (leg + feet[i] + heels[i].0 - shift).dot(heels[i].1.cross(Vec3::Y)) < 0.0
    });
    // Where leg `i`'s ankle goes, hips-relative before the move, with its
    // heel risen `angle` about the tip.
    let ankle_at = |i: usize, foot: Vec3, angle: f32| {
        let (_, hip, leg) = legs[i];
        let flat = hip + leg + foot;
        if angle == 0.0 {
            return flat;
        }
        let (tip, axis) = heels[i];
        flat + tip - Quat::from_axis_angle(axis, angle) * tip
    };
    let ankle_for = |i: usize, angle: f32| ankle_at(i, feet[i], angle);
    // The root's rise at which leg `i` just reaches its ankle with its foot
    // displaced by `foot`.
    let reach = |i: usize, foot: Vec3, angle: f32| {
        let (_, hip, leg) = legs[i];
        let shifted = ankle_at(i, foot, angle) - hip - shift;
        let horizontal = Vec3::new(shifted.x, 0.0, shifted.z).length_squared();
        shifted.y + (leg.length_squared() - horizontal).max(0.0).sqrt()
    };
    let asks = |i: usize, foot: Vec3| {
        let flat = reach(i, foot, 0.0);
        if !trailing[i] || flat >= -DROP_BEFORE_HEEL_RISE {
            return flat;
        }
        let risen = (1..=8).map(|n| reach(i, foot, rise * n as f32 / 8.0)).fold(flat, f32::max);
        risen.min(-DROP_BEFORE_HEEL_RISE).max(flat)
    };
    let lowest = |each: &dyn Fn(usize) -> f32| {
        (0..2)
            .filter(|&i| loads[i] > 0.05)
            .map(each)
            .fold(f32::MAX, f32::min)
            .min(if loads.iter().all(|&l| l <= 0.05) { 0.0 } else { f32::MAX })
    };
    // What the legs will need with their feet where they are aiming (a
    // swinging foot's landing): the height to settle toward.
    let needed = lowest(&|i| asks(i, aims[i]));
    // How high the pelvis may sit at most: each leg's need for where its
    // foot is now, plus the slack the caller gives it (a swinging foot, in
    // the air, may be left short of its arc that much).
    let ceiling = lowest(&|i| asks(i, feet[i]) + slack[i].max(0.0));
    // The caller may sit the pelvis lower than it must (`settle`), never
    // above the ceiling: higher, a foot could not be reached.
    let drop = settle(needed, ceiling).min(ceiling);
    let pivot = legs[0].1 * loads[0] + legs[1].1 * loads[1];
    if turned {
        pose.rotations[Bone::Hips] = delta_after_world_turn(pose, rig, Bone::Hips, turn);
        pose.rotations[Bone::Spine] = delta_after_world_turn(pose, rig, Bone::Spine, turn.inverse());
    }
    // Everything under the hips turned about the hips joint: the root moves
    // so the pivot stays put, then by the shift.
    let moved = shift + Vec3::Y * drop + (pivot - turn * pivot);
    pose.root_translation += moved;
    for (i, (bones, hip, leg)) in legs.into_iter().enumerate() {
        // The leg rode the turn; its socket went with the pelvis.
        let socket_now = turn * hip;
        // The least heel rise that brings the ankle into reach.
        let short = |angle: f32| (ankle_for(i, angle) - moved - socket_now).length() - leg.length();
        let angle = if !trailing[i] || short(0.0) <= 1.0e-5 {
            0.0
        } else if short(rise) > 0.0 {
            rise
        } else {
            let (mut low, mut high) = (0.0, rise);
            for _ in 0..24 {
                let middle = 0.5 * (low + high);
                if short(middle) > 0.0 { low = middle } else { high = middle }
            }
            high
        };
        keep_ankle(pose, rig, bones, turn, socket_now, socket_now + turn * leg, ankle_for(i, angle) - moved);
        if angle > 0.0 {
            // The foot pitched about its tip, rigidly.
            pose.rotations[bones[2]] =
                delta_after_world_turn(pose, rig, bones[2], Quat::from_axis_angle(heels[i].1, angle));
        }
    }
}

/// Puts `ankle`'s leg's ankle at `target` (hips-relative), its foot keeping
/// its attitude in the world: bends the knee just enough and turns the leg
/// about its hip. For stepping a foot somewhere; a target out of reach is
/// left short.
pub fn place_ankle(pose: &mut LocalPose, rig: &super::rig::RigGeometry, ankle: Bone, target: Vec3) {
    use super::rig::offset_from;
    let bones = match ankle {
        Bone::RightFoot => [Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot],
        _ => [Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot],
    };
    let hip = offset_from(pose, rig, Bone::Hips, bones[0]);
    let ankle_at = offset_from(pose, rig, Bone::Hips, bones[2]);
    keep_ankle(pose, rig, bones, Quat::IDENTITY, hip, ankle_at, target);
}

/// Puts the ankle at `target` (hips-relative) by bending the knee just
/// enough for the distance and turning the leg about its hip onto it, with
/// the foot turned back so it keeps its attitude in the world.
///
/// It keeps the knee's hinge where it is and leaves an unreachable target
/// short. `hip` and `ankle_at` are where the socket and ankle are now,
/// hips-relative, known to the caller; `carried` is the world turn the leg
/// has already been given (by its pelvis), which the foot also gives back.
fn keep_ankle(
    pose: &mut LocalPose,
    rig: &super::rig::RigGeometry,
    [socket, knee, ankle]: [Bone; 3],
    carried: Quat,
    hip: Vec3,
    ankle_at: Vec3,
    target: Vec3,
) {
    use super::rig::{delta_after_world_turn, offset_from};
    if (ankle_at - target).length_squared() < 1.0e-12 && carried.dot(Quat::IDENTITY).abs() > 1.0 - 1.0e-9 {
        return;
    }
    let knee_at = offset_from(pose, rig, Bone::Hips, knee);
    let (femur, shin) = ((knee_at - hip).length(), (ankle_at - knee_at).length());
    let hinge = (knee_at - hip).cross(ankle_at - knee_at);
    // A dead-straight leg has no hinge to bend about: it is only aimed.
    let unfold = if hinge.length_squared() < 1.0e-12 {
        Quat::IDENTITY
    } else {
        let hinge = hinge.normalize();
        // Interior knee angles now and for the distance wanted.
        let interior = |reach: f32| {
            ((femur * femur + shin * shin - reach * reach) / (2.0 * femur * shin)).clamp(-1.0, 1.0).acos()
        };
        let bend = interior((ankle_at - hip).length()) - interior((target - hip).length());
        // Whichever way about the hinge opens the knee by `bend`.
        [bend, -bend]
            .into_iter()
            .map(|angle| Quat::from_axis_angle(hinge, angle))
            .min_by(|a, b| {
                let reach =
                    |turn: &Quat| ((knee_at + *turn * (ankle_at - knee_at) - hip).length() - (target - hip).length()).abs();
                reach(a).total_cmp(&reach(b))
            })
            .unwrap_or(Quat::IDENTITY)
    };
    pose.rotations[knee] = delta_after_world_turn(pose, rig, knee, unfold);
    let reached = knee_at + unfold * (ankle_at - knee_at) - hip;
    let aim = Quat::from_rotation_arc(reached.normalize_or_zero(), (target - hip).normalize_or_zero());
    pose.rotations[socket] = delta_after_world_turn(pose, rig, socket, aim);
    pose.rotations[ankle] = delta_after_world_turn(pose, rig, ankle, (aim * unfold * carried).inverse());
}

/// A walk's step width, as a fraction of the rig's hip-socket spacing:
/// 13 cm between the feet's centrelines on `puppet_base`, where standing
/// puts them under the sockets, 22.9 cm apart.
///
/// Winter gives the constraint, not a width: in steady walking the centre
/// of mass passes just medial of each stance foot's inner border (§11.3.1,
/// Fig. 11.7). Its sway follows from the inverted pendulum (Eq. 11.3,
/// K ≈ 0.1 s²) driven by the pressure moving foot to foot, and it grows with
/// the width. On `puppet_base` (inner border 3.8 cm inside the sole's
/// centreline), 13 cm keeps the COM 4 mm medial of the border at 0.7 m/s
/// and 9 mm at 1.2 m/s; 12 cm leaves 1 mm at the slow walk, and 10 cm
/// crosses it. The standing width swayed ±3.2 cm at 1.2 m/s, twice a
/// person's.
pub const STEP_WIDTH: f32 = 0.57;

/// Brings each foot toward the midline until the feet are `width` apart
/// (between their ankles, across the rig's left), turning each leg whole
/// about its hip socket and each foot back by the same turn, so the sole
/// keeps its attitude on the ground.
///
/// The leg keeps its length, so the foot rises a little as it comes in,
/// `leg · (1 − cos θ)`: 1.3 mm on `puppet_base` for 22.9 → 13 cm.
pub fn narrow_feet(pose: &mut LocalPose, rig: &super::rig::RigGeometry, width: f32) {
    use super::rig::{delta_after_world_turn, offset_from};
    let left = rig.left();
    for (socket, ankle, side) in [(Bone::LeftUpLeg, Bone::LeftFoot, 1.0), (Bone::RightUpLeg, Bone::RightFoot, -1.0)] {
        let hip = offset_from(pose, rig, Bone::Hips, socket);
        let leg = offset_from(pose, rig, Bone::Hips, ankle) - hip;
        // The leg's new sideways component, the rest of its length shared
        // by the other two in their present proportion: a turn about the
        // rig's forward only.
        let across = side * width * 0.5 - hip.dot(left);
        let sideways = leg.dot(left);
        let rest = leg - left * sideways;
        let rest_length = rest.length();
        if rest_length < 1.0e-6 || across.abs() >= leg.length() {
            continue;
        }
        let kept = (leg.length_squared() - across * across).sqrt();
        let wanted = rest * (kept / rest_length) + left * across;
        let turn = Quat::from_rotation_arc(leg.normalize(), wanted.normalize());
        pose.rotations[socket] = delta_after_world_turn(pose, rig, socket, turn);
        pose.rotations[ankle] = delta_after_world_turn(pose, rig, ankle, turn.inverse());
    }
}

/// The step width [`STEP_WIDTH`] asks for on `rig`, metres: its fraction
/// of the distance between the hip sockets under `pose`.
pub fn step_width(pose: &LocalPose, rig: &super::rig::RigGeometry) -> f32 {
    use super::rig::offset_from;
    let sockets = offset_from(pose, rig, Bone::Hips, Bone::LeftUpLeg)
        - offset_from(pose, rig, Bone::Hips, Bone::RightUpLeg);
    sockets.dot(rig.left()).abs() * STEP_WIDTH
}

/// How far a full weight shift carries the pelvis toward the loaded foot,
/// metres.
///
/// Winter §11.2.1: side-to-side balance is the hips loading one leg and
/// unloading the other, which moves the pressure — and, held, the pelvis —
/// toward the loaded foot. A relaxed shift onto one leg brings the pelvis a
/// good part of the way over it; the feet here stand ~0.2 m apart.
pub const WEIGHT_SHIFT: f32 = 0.045;

/// How far a full weight shift drops the unloaded side of the pelvis,
/// radians (~4 degrees): the hip on the resting leg sags while the loaded
/// side's abductors hold it.
pub const WEIGHT_SHIFT_ROLL: f32 = 0.07;

/// Shifts the body's weight onto one leg: `onto` +1 is fully onto the
/// rig's left leg, −1 its right, 0 nothing.
///
/// A deliberate postural change, not quiet sway (Winter §11.2.1: a visible,
/// held shift is the hips loading one leg): the pelvis moves toward the
/// loaded foot and rolls down on the unloaded side, the spine rolls back so
/// the shoulders stay level, and both legs are re-solved to the ground where
/// the feet stood. The loaded leg straightens under the load; the unloaded
/// one, now too long for its lowered hip, relaxes its knee.
pub fn shift_weight(pose: &mut LocalPose, rig: &super::rig::RigGeometry, onto: f32) {
    use super::legik::{solve_leg_on, LegChain, LegIkConfig};
    use super::rig::{delta_after_world_turn, forward_kinematics_on, offset_from};

    let onto = onto.clamp(-1.0, 1.0);
    if onto.abs() < 1.0e-4 {
        return;
    }
    let before = forward_kinematics_on(pose, rig);
    let (left_toe, right_toe) = (before[Bone::LeftToeBase], before[Bone::RightToeBase]);
    let loaded = if onto > 0.0 { Bone::LeftUpLeg } else { Bone::RightUpLeg };
    let loaded_hip = offset_from(pose, rig, Bone::Hips, loaded);

    // Rolled about the rig's forward, so the unloaded hip goes down: a
    // positive turn about forward lifts the rig's left side.
    let roll = bevy::math::Quat::from_axis_angle(rig.forward(), onto * WEIGHT_SHIFT_ROLL);
    pose.rotations[Bone::Hips] = delta_after_world_turn(pose, rig, Bone::Hips, roll);
    pose.rotations[Bone::Spine] = delta_after_world_turn(pose, rig, Bone::Spine, roll.inverse());

    // Toward the loaded foot, and down by however much the roll lifted the
    // loaded hip, so the loaded leg can still reach the ground.
    let lifted = offset_from(pose, rig, Bone::Hips, loaded).y - loaded_hip.y;
    pose.root_translation += rig.left() * (onto * WEIGHT_SHIFT) - Vec3::Y * lifted.max(0.0);

    let config = LegIkConfig::default();
    solve_leg_on(pose, LegChain::LEFT, left_toe, &config, rig);
    solve_leg_on(pose, LegChain::RIGHT, right_toe, &config, rig);
}

/// The default standing stance on top of `base`.
pub fn stance(base: &LocalPose) -> LocalPose {
    stance_on(base, DEFAULT_KNEE_FLEX)
}

/// Multiplies a rotation onto whatever the pose already has for a bone,
/// rather than replacing it — so a stance layers onto an authored pose.
fn compose(pose: &mut LocalPose, bone: Bone, rotation: Quat) {
    pose.set_rotation(bone, pose.rotation(bone) * rotation);
}

/// How much horizontal room a foot has, given a hip height and the leg's
/// own straight reach.
///
/// This is the quantity that was zero before a stance was authored, and it
/// is what makes an IK target reachable: with the hip at `hip_height` above
/// the sole, a foot can move this far sideways before the leg would have to
/// stretch.
pub fn horizontal_reach_budget(straight_reach: f32, hip_height: f32) -> f32 {
    let squared = straight_reach * straight_reach - hip_height * hip_height;
    if squared <= 0.0 { 0.0 } else { squared.sqrt() }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::rig::{forward_kinematics, Side};

    #[test]
    fn narrowed_feet_stand_the_step_width_apart_flat_and_mirrored() {
        use crate::character::anim::gltf_rig::puppet_base;
        use crate::character::anim::rig::{accumulate_world_rotations, offset_from};
        let rig = puppet_base();
        let stood = stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        let width = step_width(&stood, &rig);
        assert!((width - 0.13).abs() < 0.002, "puppet_base's step width should be ~13 cm, got {width}");

        let mut narrowed = stood;
        narrow_feet(&mut narrowed, &rig, width);
        let left = rig.left();
        let across = |bone| offset_from(&narrowed, &rig, Bone::Hips, bone).dot(left);
        let (l, r) = (across(Bone::LeftFoot), across(Bone::RightFoot));
        assert!((l - r - width).abs() < 1.0e-4, "ankles {l:.4} / {r:.4} should be {width:.4} apart");
        assert!((l + r).abs() < 1.0e-3, "and centred on the pelvis: {l:.4} / {r:.4}");

        // The feet keep their attitude on the ground.
        let (before, after) = (accumulate_world_rotations(&stood, &rig), accumulate_world_rotations(&narrowed, &rig));
        for foot in [Bone::LeftFoot, Bone::RightFoot, Bone::LeftToeBase, Bone::RightToeBase] {
            let dot = before[foot].dot(after[foot]).abs();
            assert!(1.0 - dot < 1.0e-6, "{foot:?} tipped: 1 - |dot| = {}", 1.0 - dot);
        }
        // Each ankle rose only by what a whole-leg turn costs, the same on
        // both sides.
        let rise = |bone| offset_from(&narrowed, &rig, Bone::Hips, bone).y - offset_from(&stood, &rig, Bone::Hips, bone).y;
        let (rise_l, rise_r) = (rise(Bone::LeftFoot), rise(Bone::RightFoot));
        assert!((0.0..0.003).contains(&rise_l), "the left ankle rose {rise_l} m");
        assert!((rise_l - rise_r).abs() < 1.0e-4, "the ankles rose {rise_l} / {rise_r} m");
    }

    #[test]
    fn a_weight_shift_loads_one_leg_and_rests_the_other() {
        use crate::character::anim::anthropometry::centre_of_mass;
        use crate::character::anim::gltf_rig::puppet_base_as_rendered;
        use crate::character::anim::gait::{leg_joints, sagittal_angles};
        use crate::character::anim::rig::forward_kinematics_on;

        let rig = puppet_base_as_rendered();
        let stood = stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        let left = rig.left();
        for onto in [1.0f32, -1.0] {
            let mut shifted = stood;
            shift_weight(&mut shifted, &rig, onto);
            let (a, b) = (forward_kinematics_on(&stood, &rig), forward_kinematics_on(&shifted, &rig));

            // The feet stay where they stood.
            for toe in [Bone::LeftToeBase, Bone::RightToeBase] {
                assert!(a[toe].distance(b[toe]) < 0.003, "{onto}: {} moved {}", toe.name(), a[toe].distance(b[toe]));
            }
            // The pelvis and the body's mass move toward the loaded foot.
            let pelvis = (b[Bone::Hips] - a[Bone::Hips]).dot(left) * onto;
            assert!(pelvis > 0.03, "{onto}: pelvis moved {pelvis} toward the loaded foot");
            let com = |p: &LocalPose| (p.root_translation + centre_of_mass(p, &rig)).dot(left);
            assert!((com(&shifted) - com(&stood)) * onto > 0.025, "{onto}: the centre of mass barely moved");
            // The resting knee bends more than the loaded one.
            let knee = |ankle| sagittal_angles(&shifted, &rig, leg_joints(ankle))[2];
            let (loaded, resting) =
                if onto > 0.0 { (knee(Bone::LeftFoot), knee(Bone::RightFoot)) } else { (knee(Bone::RightFoot), knee(Bone::LeftFoot)) };
            // Measured: pelvis 4.5 cm, centre of mass 3.4 cm; loaded knee
            // 14.1 degrees, resting 25.9.
            assert!(resting > loaded + 0.05, "{onto}: resting knee {resting} against loaded {loaded}");
            // The shoulders stay level.
            let tilt = (b[Bone::LeftArm].y - b[Bone::RightArm].y) - (a[Bone::LeftArm].y - a[Bone::RightArm].y);
            assert!(tilt.abs() < 0.01, "{onto}: shoulders tilted {tilt} m");
        }
    }

    #[test]
    fn the_stance_bends_both_knees_forward_on_every_rig() {
        // THE anatomical invariant, and the one this module's own tests
        // could not express: a knee bends FORWARD. Every other leg test in
        // this crate measured the unsigned angle between thigh and shin,
        // which is identical whichever way the knee folds — so a
        // backward-bending knee passed all of them, and was reported from a
        // screenshot instead.
        //
        // Checked on a rig with REAL bind rotations, because that is the
        // only place the bug can appear: the synthetic rig's binds are all
        // identity and it faces the same way the stance was authored
        // against, so it cannot disagree.
        //
        // Measured with the bug live on `puppet_base`: `LocalPose::REST`
        // sat at `+0.0169` while `stance(REST)` — whose entire job is
        // bending the knee — sat at `-0.0157`.
        use crate::character::anim::gltf_rig;
        use crate::character::anim::rig::RigGeometry;

        for (label, rig) in [
            ("synthetic", RigGeometry::default()),
            ("puppet_base", gltf_rig::puppet_base()),
        ] {
            for side in [Side::Left, Side::Right] {
                let stood = stance_on_rig(&LocalPose::REST, DEFAULT_KNEE_FLEX, &rig);

                // Measured by the FOLD — how the shin turns relative to the
                // thigh — rather than by the knee's offset from the
                // hip-to-ankle line. Both describe the same thing on a
                // bent leg, but the offset degenerates near full extension
                // where the knee lies on that line by definition, and a
                // standing stance is only about 9 degrees off straight.
                // See `RigGeometry::knee_fold_direction`.
                let fold = rig.knee_fold_direction(&stood, side);

                assert!(
                    fold < 0.0,
                    "{label} {side:?}: the stance folded the shin {fold} along the \
                     rig's own forward relative to the thigh — positive is a knee \
                     bending backward, which no human leg does",
                );
            }
        }
    }

    /// How far the leg reaches from its hip socket to the sole, measured
    /// from the rig itself rather than assembled by hand.
    ///
    /// # A naming trap worth knowing about
    ///
    /// Each bone entity sits where its bone *starts*, positioned by its
    /// parent's offset. So `LeftUpLeg` — "the thigh" — is at the point the
    /// thigh ENDS, i.e. the knee, and the hip socket is `Hips` itself.
    ///
    /// Two earlier versions of this helper got that wrong in different
    /// ways: one summed `LeftUpLeg`'s offset as though it were the femur
    /// (reporting 0.79 m of budget on a rig with none), the other measured
    /// from `LeftUpLeg` and found a 0.508 m leg. Deriving from the bind
    /// pose's own positions, between the two joints that really are the hip
    /// and the sole, removes the guesswork.
    fn straight_leg_reach() -> f32 {
        let rest = forward_kinematics(&LocalPose::REST);
        rest[Bone::Hips].y - rest[Bone::LeftToeBase].y
    }

    #[test]
    fn the_bind_pose_has_no_horizontal_reach_budget_at_all() {
        // The problem this module exists to solve, stated as a measurement.
        // If this ever stops being true the rig has changed, and the stance
        // should be re-derived rather than assumed.
        let budget =
            horizontal_reach_budget(straight_leg_reach(), hip_to_sole(&LocalPose::REST));

        assert!(
            budget < 0.05,
            "the bind pose should be effectively singular for IK, but reported \
             {budget} m of budget",
        );
    }

    /// How far the sole sits below the hip socket under `pose`.
    ///
    /// This is the quantity a stance actually changes. `Hips` is the rig's
    /// root and the hip socket is a fixed offset from it, so neither can
    /// move under a leg rotation — bending the knee raises the *sole*
    /// toward the hips rather than lowering the hips toward the ground.
    /// Same geometry either way, and this shrink is exactly the reach
    /// budget the IK stage needs.
    ///
    /// A consumer that wants the character to visibly crouch pairs the
    /// stance with a matching `root_translation`; that is a separate
    /// concern from having a solvable leg.
    fn hip_to_sole(pose: &LocalPose) -> f32 {
        let positions = forward_kinematics(pose);
        // `Hips` IS the hip socket — see `straight_leg_reach`'s own note on
        // why `LeftUpLeg` is the knee rather than the thigh's origin.
        positions[Bone::Hips].y - positions[Bone::LeftToeBase].y
    }

    #[test]
    fn a_stance_buys_real_horizontal_reach_budget() {
        // THE acceptance criterion for this phase: enough room to place a
        // foot somewhere other than directly under the hip.
        let standing = hip_to_sole(&stance(&LocalPose::REST));
        let budget = horizontal_reach_budget(straight_leg_reach(), standing);

        // 0.05 m rather than something larger, deliberately. Budget grows
        // with the sine of the fold while the crouch grows with its cosine,
        // so reach is cheap at first and expensive later: the default 9
        // degrees buys ~0.055 m, while 0.15 m would need past 20 degrees —
        // more than a relaxed person holds, and visibly a squat.
        //
        // That is the trade-off the foot-locking literature calls "avoid
        // the dinosaur": prefer a little foot sliding over a character who
        // permanently crouches. 0.055 m is enough to place a foot outside
        // the hip, measured against a bind pose that had essentially zero —
        // and `stance_on` takes a deeper flex where a pose genuinely needs
        // one.
        assert!(
            budget > 0.05,
            "a standing stance should give a foot at least 0.05 m of horizontal room, \
             got {budget} m (hip sits {standing} m above the sole)",
        );
    }

    #[test]
    fn a_stance_shortens_the_leg_without_tilting_the_foot_off_the_ground() {
        // The three-rotation construction exists precisely so the foot
        // stays flat. Bending only the knee would swing the sole backward
        // and tip it onto its heel.
        let rest = hip_to_sole(&LocalPose::REST);
        let standing = hip_to_sole(&stance(&LocalPose::REST));

        // Barely a millimetre, and that is the point: the shortening grows
        // with the square of the fold, so a relaxed stance costs almost no
        // height while still opening up the reach budget that matters.
        //
        // Note the symmetric hip/ankle split folds each segment by `flex/2`
        // rather than `flex` — the hip's `+flex/2` cancels half the knee's
        // `-flex` — so the shortening is `L * (1 - cos(flex/2))`, about
        // half what a pure knee bend would give. That is the price of
        // keeping the shin vertical and the foot flat, and it is worth it.
        assert!(
            rest - standing > 0.001,
            "the stance should shorten the hip-to-sole distance, but it changed by only \
             {} m",
            rest - standing,
        );

        // The foot must stay level: the toe and ankle should remain at
        // roughly the same height relative to each other.
        let posed = forward_kinematics(&stance(&LocalPose::REST));
        let bind = forward_kinematics(&LocalPose::REST);

        let posed_tilt = posed[Bone::LeftToeBase].y - posed[Bone::LeftFoot].y;
        let bind_tilt = bind[Bone::LeftToeBase].y - bind[Bone::LeftFoot].y;

        assert!(
            (posed_tilt - bind_tilt).abs() < 0.02,
            "the foot should stay as flat as it was bound, but its tilt changed by {} m",
            (posed_tilt - bind_tilt).abs(),
        );
    }

    #[test]
    fn a_stance_keeps_the_shin_close_to_vertical() {
        // The hip and ankle each take half the knee's bend specifically so
        // the shin ends up back under the hip. A shin leaning far off
        // vertical would read as a lunge, not a stand.
        let positions = forward_kinematics(&stance(&LocalPose::REST));

        let shin = (positions[Bone::LeftFoot] - positions[Bone::LeftLeg]).normalize();
        let lean = shin.dot(Vec3::NEG_Y).acos().to_degrees();

        assert!(
            lean < 15.0,
            "the shin should stay near vertical, but leans {lean} degrees",
        );
    }

    #[test]
    fn a_stance_bends_both_knees_by_the_authored_amount() {
        let flex = 0.2;
        let posed = stance_on(&LocalPose::REST, flex);

        for knee in [Bone::LeftLeg, Bone::RightLeg] {
            let angle = posed.rotation(knee).angle_between(Quat::IDENTITY);
            assert!(
                (angle - flex).abs() < 1.0e-5,
                "{} should bend by {flex} rad, got {angle}",
                knee.name(),
            );
        }
    }

    #[test]
    fn both_legs_bend_identically() {
        // An asymmetric stance would read as a limp. Idle asymmetry is
        // added later, deliberately, by the phase layer — not here.
        let posed = stance(&LocalPose::REST);

        for (left, right) in [
            (Bone::LeftUpLeg, Bone::RightUpLeg),
            (Bone::LeftLeg, Bone::RightLeg),
            (Bone::LeftFoot, Bone::RightFoot),
        ] {
            assert!(
                posed.rotation(left).abs_diff_eq(posed.rotation(right), 1.0e-6),
                "{} and {} should bend identically",
                left.name(),
                right.name(),
            );
        }
    }

    #[test]
    fn the_knee_bends_the_way_a_knee_actually_bends() {
        // A knee that hinges the wrong way is instantly, viscerally wrong,
        // and is the exact sign error that cost the superseded module real
        // debugging time. The heel must move BACKWARD, never forward.
        //
        // The rig faces -Z, so backward is +Z.
        let rest = forward_kinematics(&LocalPose::REST);
        let posed = forward_kinematics(&stance(&LocalPose::REST));

        let knee_shift = posed[Bone::LeftLeg].z - rest[Bone::LeftLeg].z;
        assert!(
            knee_shift < 0.0,
            "the knee should travel FORWARD (-Z) as it bends, but moved {knee_shift} in Z \
             — the bend axis sign is inverted",
        );
    }

    #[test]
    fn a_zero_flex_stance_is_exactly_the_base_pose() {
        let mut base = LocalPose::REST;
        base.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, 0.4));

        let posed = stance_on(&base, 0.0);

        for &bone in Bone::ALL.iter() {
            assert_eq!(
                posed.rotation(bone),
                base.rotation(bone),
                "{} should be untouched at zero flex",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_stance_leaves_the_upper_body_alone() {
        // It composes onto an authored pose, so the arms and spine it was
        // given must survive untouched.
        let mut base = LocalPose::REST;
        let arm = Quat::from_axis_angle(Vec3::Z, 1.2);
        base.set_rotation(Bone::LeftArm, arm);
        base.set_rotation(Bone::Spine, Quat::from_axis_angle(Vec3::X, 0.3));

        let posed = stance(&base);

        assert_eq!(posed.rotation(Bone::LeftArm), arm, "the arm must be untouched");
        assert_eq!(
            posed.rotation(Bone::Spine),
            base.rotation(Bone::Spine),
            "the spine must be untouched",
        );
    }

    #[test]
    fn a_stance_preserves_every_bone_length() {
        // Free in rotation space, but worth pinning: this is the invariant
        // that the superseded position-space module could only approximate.
        let positions = forward_kinematics(&stance(&LocalPose::REST));

        for &bone in Bone::ALL.iter() {
            let Some(parent) = bone.parent() else { continue };
            let rest_length = bone.t_pose_offset().length();
            let posed_length = (positions[bone] - positions[parent]).length();

            assert!(
                (posed_length - rest_length).abs() < 1.0e-5,
                "{} should stay {rest_length} m from {}, got {posed_length}",
                bone.name(),
                parent.name(),
            );
        }
    }

    #[test]
    fn a_deeper_stance_buys_more_budget() {
        // Monotonic, so the knob behaves predictably when the IK stage
        // needs more room.
        let budget_at = |flex: f32| {
            horizontal_reach_budget(
                straight_leg_reach(),
                hip_to_sole(&stance_on(&LocalPose::REST, flex)),
            )
        };

        let shallow = budget_at(0.10);
        let default = budget_at(DEFAULT_KNEE_FLEX);
        let deep = budget_at(0.30);

        assert!(shallow < default, "{shallow} should be less than {default}");
        assert!(default < deep, "{default} should be less than {deep}");
    }

    #[test]
    fn the_default_flex_stays_in_the_range_a_person_actually_stands_at() {
        // Guards against quietly maximising this for IK headroom and
        // ending up with a permanent crouch.
        let degrees = DEFAULT_KNEE_FLEX.to_degrees();
        assert!(
            (5.0..=15.0).contains(&degrees),
            "the default knee flex should be a relaxed 5-15 degrees, got {degrees}",
        );
    }

    #[test]
    fn the_reach_budget_helper_is_zero_when_the_leg_is_exactly_straight() {
        assert_eq!(horizontal_reach_budget(0.94, 0.94), 0.0);
        assert_eq!(horizontal_reach_budget(0.94, 1.5), 0.0, "and clamps rather than NaN-ing");
        assert!(horizontal_reach_budget(0.94, 0.90) > 0.2);
    }
}
