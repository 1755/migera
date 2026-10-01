//! Joint drives: a real joint torque between two bodies, the reaction on the
//! parent included, integrated implicitly every physics substep (plan step
//! 4b).
//!
//! # Why not the ragdoll's PD
//!
//! [`super::ragdoll`]'s controller turns each body toward its WORLD target
//! with an angular acceleration on that body alone. Nothing pushes back on
//! the body it hangs from, so it is an invisible hand per body, not a
//! muscle, and it holds the body up only by switching gravity off
//! (`GravityScale = 1 − strength`). Unpinned under real gravity it buckled
//! at 0.5 s, or at 16 Hz held ~1.1 s and then slid its feet apart (see the
//! knowledge note on an unpinned ragdoll).
//!
//! A standing body carries its weight through its joints: a torque `+τ` on
//! the child and `−τ` on the parent, in N·m, sized to the load. That is
//! unstable explicitly integrated at the stiffness standing needs against a
//! light foot. So it is solved as a soft constraint, implicitly, the way
//! avian's own motors and Box2D's soft step do, and inside avian's substep
//! loop: [`drive_impulse`] is stable at any stiffness and step.
//!
//! # The drive
//!
//! For one substep `h`, with `e` the child's rotation error from its target
//! relative to the parent (world scaled-angle-axis), `v` the child's spin
//! relative to the parent, and `K` the two bodies' inverse angular inertias
//! summed (world), the impulse on the child is
//!
//! ```text
//!   P = −(1 + c·K)⁻¹ (h·kp·e + c·v),   c = h·kd + h²·kp
//! ```
//!
//! which is implicit Euler on `kp·e + kd·v` taken at the step's END: the
//! spring sees where the joint will be, so it cannot overshoot by being
//! late. The parent receives `−P`.

use avian3d::dynamics::solver::solver_body::{SolverBody, SolverBodyInertia};
use avian3d::prelude::*;
use bevy::math::{Mat3, Quat, Vec2, Vec3};
use bevy::prelude::*;

use super::math::quat_ext::{neighborhood, to_scaled_angle_axis};
use super::ragdoll::Ragdoll;
use super::ragdoll_plugin::{nearest_simulated_ancestor, JointTarget, KinematicRoot};
use crate::character::skeleton::Bone;

/// A joint drive's stiffness per kilogram of body mass, N·m/rad/kg, for
/// the load-bearing chain (see [`drive_share`]). The drives only correct:
/// the weight is fed forward. Unpinned on `puppet_base`, 10 and 40 stood
/// alike (knees 5-7° off their targets either way, the stiffness the light
/// bodies let through), 10 at the lightest.
pub const DRIVE_STIFFNESS_PER_KG: f32 = 10.0;

/// How far outside the planted soles the capture point may run before a
/// body standing on its own feet falls, metres: past the edge the COP
/// cannot bring it back, and it has no step to take yet.
pub const UNCATCHABLE: f32 = 0.02;

/// A drive's damping per unit stiffness, seconds.
pub const DRIVE_DAMPING_SECONDS: f32 = 0.1;

/// The share of [`DRIVE_STIFFNESS_PER_KG`] each joint gets: the
/// load-bearing chain (legs, spine) all of it, the head and arms less.
pub fn drive_share(bone: Bone) -> f32 {
    match bone {
        Bone::Neck | Bone::Head => 0.3,
        Bone::LeftArm | Bone::RightArm => 0.2,
        Bone::LeftForeArm | Bone::RightForeArm | Bone::LeftHand | Bone::RightHand => 0.1,
        _ => 1.0,
    }
}

/// A body driven toward its [`JointTarget`] relative to its parent's, by a
/// joint torque between the two. On the child body.
#[derive(Component, Debug, Clone, Copy, PartialEq, Reflect)]
#[reflect(Component)]
pub struct JointDrive {
    /// A torque on the child held for the whole step, N·m, world (the
    /// parent gets its negative): the weight this joint carries, fed
    /// forward so the stiffness only corrects.
    pub feedforward: Vec3,
    /// The parent body (the nearest simulated ancestor's).
    pub parent: Entity,
    /// N·m per radian of error.
    pub stiffness: f32,
    /// N·m per rad/s of relative spin.
    pub damping: f32,
    /// Whether the ground holds the child (a foot flat on the floor): the
    /// drive is then sized on the parent's inertia alone.
    pub child_grounded: bool,
    /// For a planted foot: what the ground can hold the ankle with, the
    /// sole's corners (horizontal `x`, `z`), the joint point, and the
    /// weight on the foot, N. The ankle's torque is held to what keeps
    /// its pressure inside the sole.
    pub sole: Option<([Vec2; 4], Vec3, f32)>,
}

/// The angular impulse on the child for one substep `h` (the parent gets
/// its negative): see the module doc. `inverse_inertia` is both bodies'
/// world inverse angular inertias summed.
pub fn drive_impulse(error: Vec3, relative_velocity: Vec3, inverse_inertia: Mat3, stiffness: f32, damping: f32, h: f32) -> Vec3 {
    let c = h * damping + h * h * stiffness;
    let a = Mat3::IDENTITY + inverse_inertia * c;
    if a.determinant().abs() < 1.0e-12 {
        return Vec3::ZERO;
    }
    -(a.inverse() * (error * (h * stiffness) + relative_velocity * c))
}

/// The child's rotation error from where `parent_target⁻¹ · child_target`
/// would put it under the parent as it is now: world scaled-angle-axis.
pub fn drive_error(child: Quat, parent: Quat, child_target: Quat, parent_target: Quat) -> Vec3 {
    let wanted = (parent * (parent_target.inverse() * child_target)).normalize();
    let wanted = neighborhood(child, wanted);
    to_scaled_angle_axis(child * wanted.inverse())
}

/// Applies every [`JointDrive`] for one substep. Runs in avian's
/// `SubstepSchedule`, before velocities are integrated, on the solver's own
/// body state: `Rotation` is the step's start there, `delta_rotation` what
/// the substeps have added since.
pub(crate) fn apply_joint_drives(
    drives: Query<(Entity, &JointDrive, &JointTarget)>,
    targets: Query<&JointTarget>,
    mut bodies: Query<(&mut SolverBody, &SolverBodyInertia, &Rotation)>,
    time: Res<Time>,
) {
    let h = time.delta_secs();
    if h <= 0.0 {
        return;
    }
    for (child, drive, child_target) in &drives {
        let Ok(parent_target) = targets.get(drive.parent) else { continue };
        let Ok([(mut child_body, child_inertia, child_rotation), (mut parent_body, parent_inertia, parent_rotation)]) =
            bodies.get_many_mut([child, drive.parent])
        else {
            continue;
        };
        let child_now = (child_body.delta_rotation.0 * child_rotation.0).normalize();
        let parent_now = (parent_body.delta_rotation.0 * parent_rotation.0).normalize();
        let error = drive_error(child_now, parent_now, child_target.target, parent_target.target);
        let relative = child_body.angular_velocity - parent_body.angular_velocity;
        let (child_inverse, parent_inverse) =
            (child_inertia.effective_inv_angular_inertia().to_mat3(), parent_inertia.effective_inv_angular_inertia().to_mat3());
        let sized_on = if drive.child_grounded { parent_inverse } else { child_inverse + parent_inverse };
        let mut impulse = drive_impulse(error, relative, sized_on, drive.stiffness, drive.damping, h) + drive.feedforward * h;
        if drive.child_grounded
            && let Some((sole, _, weight)) = drive.sole
        {
            impulse = within_sole(impulse / h, &sole, weight) * h;
        }
        if !drive.child_grounded {
            child_body.angular_velocity += child_inverse * impulse;
        }
        parent_body.angular_velocity -= parent_inverse * impulse;
    }
}

/// Gives a character standing on its own feet (`Ragdoll::carries_itself`)
/// its drives and frees its root; takes the drives away, and the planted
/// feet's dominance with them, when it no longer does (a fall).
pub(crate) fn manage_joint_drives(
    mut commands: Commands,
    characters: Query<&Ragdoll>,
    masses: Query<&ComputedMass>,
    drives: Query<&JointDrive>,
    roots: Query<(), With<KinematicRoot>>,
) {
    for ragdoll in &characters {
        let bodies: Vec<(Bone, Entity)> = ragdoll.bodies.iter().filter_map(|(bone, body)| Some((bone, (*body)?))).collect();
        if ragdoll.carries_itself() {
            let total: f32 = bodies.iter().filter_map(|(_, body)| masses.get(*body).ok()).map(|mass| mass.value()).sum();
            for &(bone, body) in &bodies {
                if bone == Bone::Hips && roots.get(body).is_ok() {
                    commands.entity(body).remove::<KinematicRoot>().insert(RigidBody::Dynamic);
                }
                let Some(parent) = nearest_simulated_ancestor(bone, ragdoll).and_then(|parent| ragdoll.bodies[parent]) else { continue };
                if drives.get(body).is_err() {
                    let stiffness = DRIVE_STIFFNESS_PER_KG * total * drive_share(bone);
                    commands.entity(body).insert(JointDrive {
                        feedforward: Vec3::ZERO,
                        parent,
                        stiffness,
                        damping: stiffness * DRIVE_DAMPING_SECONDS,
                        child_grounded: false,
                        sole: None,
                    });
                }
            }
        } else {
            for &(_, body) in &bodies {
                if drives.get(body).is_ok() {
                    commands.entity(body).remove::<(JointDrive, Dominance)>();
                }
            }
        }
    }
}

/// Whether `body` touches a static body (the ground).
fn touches_ground(body: Entity, contacts: &ContactGraph, kinds: &Query<&RigidBody>) -> bool {
    contacts.contact_pairs_with(body).any(|pair| {
        let other = if pair.collider1 == body { pair.collider2 } else { pair.collider1 };
        pair.is_touching() && kinds.get(other).is_ok_and(|kind| *kind == RigidBody::Static)
    })
}

/// What [`carry_weight`] reads of each body.
type BodyState = (
    &'static Position,
    &'static Rotation,
    &'static ComputedMass,
    &'static ComputedCenterOfMass,
    &'static LinearVelocity,
    Option<&'static Collider>,
);

/// Once per physics step, for every character carrying itself: which feet
/// are planted, and the weight each joint carries (`JointDrive::feedforward`).
///
/// # Planted feet
///
/// A foot touching the ground is planted: [`Dominance`] 1, so the joint
/// above moves the leg and never pushes the foot, and its ankle drive is
/// carried by the ground (`child_grounded`). avian solves its joints after
/// its contacts in every substep, each correction split by inverse mass,
/// so a 1 kg foot under the body took nearly all of it: unpinned, both
/// soles sank 52 mm into the floor and tilted 10° in 9 frames. Pushing the
/// ankle's reaction into a planted foot instead spun it out of the floor.
///
/// # The weight each joint carries
///
/// The drives' correcting stiffness is capped by the two light bodies they
/// are sized on, whatever the gain: the body above an ankle cannot turn
/// with its shin alone. So each joint is fed the gravity moment of the side
/// the ground does not hold, about its joint point:
/// - its child's side (an arm, the head, the trunk above a spine joint, a
///   leg in the air) hangs from it;
/// - on a planted leg, the side above: that leg above the joint, and the
///   leg's share of everything on no planted leg, by where the centre of
///   mass stands between the feet.
///
/// With nothing planted, nothing is fed: a body in the air carries no
/// weight.
///
/// # Balance (plan step 4c)
///
/// Winter's pendulum law, as the standing balance poses it
/// (`balance::Balance`), on the measured centre of mass: the pressure goes
/// to `COP = COM + (COM − rest)·k·ω² + v·2ζω·k`, held inside the planted
/// soles. `rest` is where the COM stood over the feet when both were first
/// planted; `k` the COM's height over the ankles over `g`. Each planted
/// ankle then carries its load as pushed up at its own share of that COP
/// (Winter's ankle moment, `W·(COP − ankle)`), and each leg's share of
/// everything else follows where the COP stands between the feet: the
/// hips' load/unload (§11.2.1).
pub(crate) fn carry_weight(
    mut characters: Query<&mut Ragdoll>,
    mut drives: Query<&mut JointDrive>,
    bodies: Query<BodyState>,
    kinds: Query<&RigidBody>,
    contacts: Res<ContactGraph>,
    mut commands: Commands,
    gravity: Res<Gravity>,
) {
    const FEET: [Bone; 2] = [Bone::LeftFoot, Bone::RightFoot];
    for mut ragdoll in &mut characters {
        if !ragdoll.carries_itself() {
            ragdoll.stand_rest = None;
            continue;
        }
        let all: Vec<Bone> = Bone::ALL.iter().copied().filter(|&bone| ragdoll.bodies[bone].is_some()).collect();
        // Each body's centre of mass and mass, and its bone's joint point.
        let mut state = super::rig::BoneSet::splat((Vec3::ZERO, 0.0f32, Vec3::ZERO));
        let (mut momentum, mut total) = (Vec3::ZERO, 0.0f32);
        for &bone in &all {
            let Ok((position, rotation, mass, centre, velocity, _)) = bodies.get(ragdoll.bodies[bone].unwrap()) else { return };
            state[bone] = (position.0 + rotation.0 * centre.0, mass.value(), position.0 - rotation.0 * ragdoll.body_offsets[bone]);
            momentum += velocity.0 * mass.value();
            total += mass.value();
        }
        let below = |top: Bone| -> Vec<Bone> {
            all.iter()
                .copied()
                .filter(|&bone| std::iter::successors(Some(bone), |b| b.parent()).any(|b| b == top))
                .collect()
        };
        let legs = [below(Bone::LeftUpLeg), below(Bone::RightUpLeg)];
        let planted: Vec<usize> = (0..2)
            .filter(|&leg| ragdoll.bodies[FEET[leg]].is_some_and(|foot| touches_ground(foot, &contacts, &kinds)))
            .collect();
        for (leg, foot) in FEET.iter().enumerate() {
            let Some(body) = ragdoll.bodies[*foot] else { continue };
            let down = planted.contains(&leg);
            if let Ok(mut drive) = drives.get_mut(body)
                && drive.child_grounded != down
            {
                drive.child_grounded = down;
                commands.entity(body).insert(Dominance(down as i8));
            }
        }
        let flat = |v: Vec3| Vec2::new(v.x, v.z);
        let com = all.iter().fold(Vec3::ZERO, |sum, &b| sum + state[b].0 * state[b].1) / total;
        let velocity = momentum / total;
        // Each planted sole's footprint, its block's bottom corners, and
        // the height of the ground under it.
        let soles: Vec<(Vec<Vec2>, f32)> = planted
            .iter()
            .map(|&leg| {
                let Some(Ok((position, rotation, _, _, _, Some(collider)))) = ragdoll.bodies[FEET[leg]].map(|body| bodies.get(body)) else {
                    return (vec![flat(state[FEET[leg]].2)], state[FEET[leg]].2.y);
                };
                let mut corners = Vec::new();
                if let Some(compound) = collider.shape_scaled().as_compound() {
                    for (iso, shape) in compound.shapes() {
                        let Some(cuboid) = shape.as_cuboid() else { continue };
                        let half = Vec3::new(cuboid.half_extents.x, cuboid.half_extents.y, cuboid.half_extents.z);
                        let (offset, turn) = (Vec3::new(iso.translation.x, iso.translation.y, iso.translation.z), iso.rotation);
                        for corner in 0..8 {
                            let sign = |bit: u32| if corner & (1 << bit) == 0 { -1.0 } else { 1.0 };
                            let local = Vec3::new(sign(0) * half.x, sign(1) * half.y, sign(2) * half.z);
                            corners.push(position.0 + rotation.0 * (offset + turn * local));
                        }
                    }
                }
                let lowest = corners.iter().map(|c| c.y).fold(f32::MAX, f32::min);
                let bottom: Vec<Vec2> = corners.iter().filter(|c| c.y < lowest + 0.005).map(|&c| flat(c)).collect();
                if bottom.is_empty() { (vec![flat(state[FEET[leg]].2)], state[FEET[leg]].2.y) } else { (convex_hull(bottom), lowest) }
            })
            .collect();
        // Where the pressure goes: Winter's law on the measured COM, held
        // inside the soles.
        let mut cop = flat(com);
        if !planted.is_empty() {
            let middle = planted.iter().fold(Vec3::ZERO, |sum, &leg| sum + state[FEET[leg]].2) / planted.len() as f32;
            if planted.len() == 2 && ragdoll.stand_rest.is_none() {
                ragdoll.stand_rest = Some(Vec3::new(com.x - middle.x, 0.0, com.z - middle.z));
            }
            let rest = flat(middle + ragdoll.stand_rest.unwrap_or(Vec3::new(com.x - middle.x, 0.0, com.z - middle.z)));
            let ankles = planted.iter().map(|&leg| state[FEET[leg]].2.y).sum::<f32>() / planted.len() as f32;
            let k = ((com.y - ankles) / gravity.0.length().max(1.0e-6)).max(0.0);
            let (w, zeta) = (super::balance::RECOVERY_FREQUENCY, super::balance::RECOVERY_DAMPING);
            let wanted = flat(com) + (flat(com) - rest) * (k * w * w) + flat(velocity) * (2.0 * zeta * w * k);
            let hull = convex_hull(soles.iter().flat_map(|(sole, _)| sole.iter().copied()).collect());
            // Beyond what the feet can catch: the capture point (Hof) out of
            // the soles. With no step to take on its own feet yet, it
            // falls, and the fall's own machinery takes the body.
            let capture = flat(com) + flat(velocity) * k.sqrt();
            let (edge, caught) = nearest_in_polygon(capture, &hull);
            if !caught && edge.distance(capture) > UNCATCHABLE {
                ragdoll.fall(super::ragdoll::FALL_TONE, super::ragdoll::FALL_DAMPING);
                continue;
            }
            let (held, inside) = nearest_in_polygon(wanted, &hull);
            let centre = hull.iter().copied().sum::<Vec2>() / hull.len().max(1) as f32;
            cop = if inside { held } else { held + (centre - held).normalize_or_zero() * super::balance::SUPPORT_MARGIN };
        }
        // Each planted leg's share of what no planted leg carries, by where
        // the pressure stands between the feet.
        let upper: Vec<Bone> = all.iter().copied().filter(|b| !planted.iter().any(|&leg| legs[leg].contains(b))).collect();
        let mut share = [0.0f32; 2];
        match planted[..] {
            [leg] => share[leg] = 1.0,
            [left, right] => {
                let centre = |i: usize| soles[i].0.iter().copied().sum::<Vec2>() / soles[i].0.len() as f32;
                let (l, r) = (centre(0), centre(1));
                let t = ((cop - l).dot(r - l) / (r - l).length_squared().max(1.0e-6)).clamp(0.0, 1.0);
                share[left] = 1.0 - t;
                share[right] = t;
            }
            _ => {}
        }
        // The body's horizontal acceleration under that pressure (the
        // pendulum: g·(COM − COP)/height), carried by every part as
        // d'Alembert's effective gravity `g − a`.
        let effective = if planted.is_empty() {
            gravity.0
        } else {
            let floor = soles.iter().map(|(_, y)| *y).fold(f32::MAX, f32::min);
            let height = (com.y - floor).max(0.1);
            let lean = flat(com) - cop;
            gravity.0 - Vec3::new(lean.x, 0.0, lean.y) * (gravity.0.length() / height)
        };
        for &bone in &all {
            let Ok(mut drive) = drives.get_mut(ragdoll.bodies[bone].unwrap()) else { continue };
            let joint = state[bone].2;
            let moment = |set: &[Bone]| set.iter().fold(Vec3::ZERO, |sum, &b| sum + (state[b].0 - joint).cross(effective * state[b].1));
            let subtree = below(bone);
            drive.sole = None;
            drive.feedforward = if planted.is_empty() {
                Vec3::ZERO
            } else if let Some(leg) = planted.iter().copied().find(|&leg| legs[leg].contains(&bone)) {
                // On a planted leg: the ground pushes on this foot at its
                // share of the pressure (the point of its sole nearest the
                // COP), with its load's weight and the force that moves it.
                // The joint carries that push's moment about itself, less
                // the segments below it.
                let index = planted.iter().position(|&p| p == leg).unwrap();
                let (at, _) = nearest_in_polygon(cop, &soles[index].0);
                let at = Vec3::new(at.x, soles[index].1, at.y);
                let load = legs[leg].iter().map(|&b| state[b].1).sum::<f32>() + share[leg] * upper.iter().map(|&b| state[b].1).sum::<f32>();
                if bone == FEET[leg] {
                    drive.sole = <[Vec2; 4]>::try_from(soles[index].0.iter().map(|&c| c - flat(joint)).collect::<Vec<_>>())
                        .ok()
                        .map(|corners| (corners, joint, load * -effective.y));
                }
                (at - joint).cross(effective * load) - moment(&subtree)
            } else {
                -moment(&subtree)
            };
        }
    }
}

/// `torque` on a planted foot (N·m, world) held to what the ground can
/// give it: the pressure it implies, `weight` (N) pushing up at a
/// horizontal offset `d` from the ankle (`τ = d × (0, −W, 0)`), kept inside
/// `sole` (the sole's corners as offsets from the ankle, counter-clockwise).
/// The twist about the vertical is left alone.
pub fn within_sole(torque: Vec3, sole: &[Vec2; 4], weight: f32) -> Vec3 {
    if weight <= 0.0 {
        return Vec3::new(0.0, torque.y, 0.0);
    }
    // τx = dz·W, τz = −dx·W.
    let offset = Vec2::new(-torque.z / weight, torque.x / weight);
    let (held, _) = nearest_in_polygon(offset, sole);
    Vec3::new(held.y * weight, torque.y, -held.x * weight)
}

/// The convex hull of `points` (horizontal, `x` and `z` as a `Vec2`),
/// counter-clockwise, by Andrew's monotone chain.
pub fn convex_hull(mut points: Vec<Vec2>) -> Vec<Vec2> {
    points.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    points.dedup_by(|a, b| a.distance_squared(*b) < 1.0e-12);
    if points.len() < 3 {
        return points;
    }
    let cross = |o: Vec2, a: Vec2, b: Vec2| (a - o).perp_dot(b - o);
    let mut hull: Vec<Vec2> = Vec::with_capacity(points.len() * 2);
    for pass in 0..2 {
        let start = hull.len();
        let ordered: Box<dyn Iterator<Item = &Vec2>> = if pass == 0 { Box::new(points.iter()) } else { Box::new(points.iter().rev()) };
        for &p in ordered {
            while hull.len() >= start + 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
    }
    hull
}

/// The point of the convex polygon `hull` (counter-clockwise) nearest `p`,
/// and whether `p` was inside it.
pub fn nearest_in_polygon(p: Vec2, hull: &[Vec2]) -> (Vec2, bool) {
    if hull.len() < 3 {
        return (hull.first().copied().unwrap_or(p), false);
    }
    let edges = || hull.iter().zip(hull.iter().cycle().skip(1));
    if edges().all(|(&a, &b)| (b - a).perp_dot(p - a) >= 0.0) {
        return (p, true);
    }
    let nearest = edges()
        .map(|(&a, &b)| {
            let t = ((p - a).dot(b - a) / (b - a).length_squared().max(1.0e-12)).clamp(0.0, 1.0);
            a + (b - a) * t
        })
        .min_by(|x, y| x.distance_squared(p).total_cmp(&y.distance_squared(p)))
        .unwrap_or(p);
    (nearest, false)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_ankle_torque_is_held_to_what_the_sole_can_give() {
        // A 700 N foot, its sole 0.06 m behind the ankle to 0.18 ahead (+z)
        // and ±0.05 across (x). Without the hold, a glued foot let the
        // ankle push as if the pressure stood anywhere, and a 1.2 m/s push
        // was "caught".
        let sole = [Vec2::new(-0.05, -0.06), Vec2::new(0.05, -0.06), Vec2::new(0.05, 0.18), Vec2::new(-0.05, 0.18)];
        let weight = 700.0;
        // Pressure 0.1 m ahead: τx = dz·W = 70, inside, untouched.
        let inside = Vec3::new(70.0, 3.0, 0.0);
        assert!((within_sole(inside, &sole, weight) - inside).length() < 1.0e-3);
        // 0.3 m ahead is past the toe: held at 0.18 (τx = 126); the twist
        // is kept.
        let held = within_sole(Vec3::new(210.0, 3.0, 0.0), &sole, weight);
        assert!((held - Vec3::new(126.0, 3.0, 0.0)).length() < 1.0e-3, "{held}");
        // 0.1 m to the side (τz = −dx·W = −70): held at 0.05.
        let held = within_sole(Vec3::new(0.0, 0.0, -70.0), &sole, weight);
        assert!((held - Vec3::new(0.0, 0.0, -35.0)).length() < 1.0e-3, "{held}");
    }

    #[test]
    fn a_polygon_holds_what_is_inside_and_clamps_what_is_not() {
        // Two soles side by side, as boxes' corners, and a stray inside.
        let soles = vec![
            Vec2::new(-0.15, -0.05), Vec2::new(-0.05, -0.05), Vec2::new(-0.15, 0.20), Vec2::new(-0.05, 0.20),
            Vec2::new(0.05, -0.05), Vec2::new(0.15, -0.05), Vec2::new(0.05, 0.20), Vec2::new(0.15, 0.20),
            Vec2::new(0.0, 0.1),
        ];
        let hull = convex_hull(soles);
        assert_eq!(hull.len(), 4, "the two soles' hull is their outer box: {hull:?}");
        let (at, inside) = nearest_in_polygon(Vec2::new(0.0, 0.05), &hull);
        assert!(inside && at == Vec2::new(0.0, 0.05));
        let (at, inside) = nearest_in_polygon(Vec2::new(0.0, 0.5), &hull);
        assert!(!inside && (at - Vec2::new(0.0, 0.2)).length() < 1.0e-6, "{at}");
        let (at, _) = nearest_in_polygon(Vec2::new(0.4, -0.3), &hull);
        assert!((at - Vec2::new(0.15, -0.05)).length() < 1.0e-6, "a corner: {at}");
    }

    /// One body on a fixed parent, driven toward zero error; returns the
    /// angle each step.
    fn settle(stiffness: f32, damping: f32, inertia: f32, h: f32, steps: usize) -> Vec<f32> {
        let inverse = Mat3::from_diagonal(Vec3::splat(1.0 / inertia));
        let (mut angle, mut spin) = (0.5f32, 0.0f32);
        (0..steps)
            .map(|_| {
                let impulse = drive_impulse(Vec3::X * angle, Vec3::X * spin, inverse, stiffness, damping, h);
                spin += impulse.x / inertia;
                angle += spin * h;
                angle
            })
            .collect()
    }

    #[test]
    fn a_drive_is_stable_at_any_stiffness_and_step() {
        // Explicitly, kp/I·h² over 4 diverges. Here a 1 kg·cm² foot on
        // 1e6 N·m/rad at a 64 Hz step (kp/I·h² ≈ 2.4e6) still only settles.
        for (stiffness, damping, inertia, h) in
            [(700.0, 70.0, 0.005, 1.0 / 768.0), (1.0e6, 1.0e4, 1.0e-4, 1.0 / 64.0), (10.0, 0.0, 1.0, 1.0 / 64.0)]
        {
            let angles = settle(stiffness, damping, inertia, h, 2000);
            let worst = angles.iter().fold(0.0f32, |a, b| a.max(b.abs()));
            assert!(worst <= 0.5 + 1.0e-6, "kp {stiffness}, I {inertia}, h {h}: grew to {worst}");
            if damping > 0.0 {
                assert!(angles.last().unwrap().abs() < 1.0e-3, "kp {stiffness}: did not settle, {}", angles.last().unwrap());
            }
        }
    }

    #[test]
    fn a_drive_matches_the_spring_at_small_steps() {
        // At a small step, the implicit drive is the plain damped spring:
        // critically damped (kd = 2√(kp·I)) it never overshoots, and at
        // ωt = 3 it has (1 + ωt)e^{-ωt} = 4e⁻³ ≈ 0.199 of its error left.
        let (stiffness, inertia): (f32, f32) = (100.0, 0.25);
        let omega = (stiffness / inertia).sqrt();
        let damping = 2.0 * (stiffness * inertia).sqrt();
        let h = 1.0e-4;
        let angles = settle(stiffness, damping, inertia, h, (3.0 / omega / h) as usize);
        assert!(angles.iter().all(|a| *a > -1.0e-6), "a critically damped drive overshot");
        let expected = 0.5 * 4.0 * (-3.0f32).exp();
        assert!((angles.last().unwrap() - expected).abs() < 2.0e-3, "after ωt = 3: {} against {expected}", angles.last().unwrap());
    }

    #[test]
    fn the_error_is_measured_relative_to_the_parent() {
        // A parent turned 30° about Y carries the child's target with it:
        // a child turned the same is on target.
        let turn = Quat::from_rotation_y(0.5);
        let error = drive_error(turn, turn, Quat::IDENTITY, Quat::IDENTITY);
        assert!(error.length() < 1.0e-5, "{error}");
        // Unturned, the child is 0.5 rad off, about the turn's axis back.
        let error = drive_error(Quat::IDENTITY, turn, Quat::IDENTITY, Quat::IDENTITY);
        assert!((error - Vec3::NEG_Y * 0.5).length() < 1.0e-5, "{error}");
    }
}
