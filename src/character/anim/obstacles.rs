//! What a foot must not stand in or swing through: chair legs, table legs,
//! walls, props.
//!
//! The foot IK asks, for each foot (its middle line, heel to tip, in the
//! world), how far to move it so that line keeps a clearance from every
//! obstacle at foot height ([`FootObstacles::clear`]): a capsule query. It
//! moves the toe's target by that, and a planted foot's lock with it, so a
//! swinging foot lands beside the obstacle and a planted one the body turns
//! about (`footlock`) is held off it.
//!
//! Asked in the IK stage, about the toe target the leg is solved to, the
//! clearance is a property of the foot ([`FOOT_CLEARANCE`]). Worked out on
//! the pose the walker wrote instead, it had to be 12 cm: the sprung leg
//! trails that pose by 4-7 cm swinging fast.
//!
//! Like `ground::GroundProbe`, implemented by the consumer: [`Footprints`]
//! are boxes on the floor (a chair, a table's legs), for tests and simple
//! scenes; a physics world answers the same question with a capsule query.

use bevy::math::{Vec2, Vec3};
use bevy::prelude::Component;

/// A foot's heel behind its ankle joint and its tip beyond its toe joint,
/// metres (puppet_base's foot, ~0.25 m long).
pub const HEEL_BEHIND_ANKLE: f32 = 0.06;
pub const TIP_BEYOND_TOE: f32 = 0.05;
/// How far a foot's middle line keeps from an obstacle, metres: half its
/// ~0.09 m width, the ~2 cm the drawn foot falls short of the IK's toe
/// target (its heel turns as the leg solve places the ankle), and a
/// centimetre to spare. At 5 cm the drawn foot came within 3.2 cm of a
/// chair's footprint and 1.9 cm into its leg.
pub const FOOT_CLEARANCE: f32 = 0.08;

/// A foot's middle line, heel to tip, from its toe joint `toe`, pointing
/// `along` (horizontal, unit), its ankle `ankle_back` behind the toe joint.
pub fn foot_line(toe: Vec3, along: Vec3, ankle_back: f32) -> (Vec3, Vec3) {
    (toe - along * (ankle_back + HEEL_BEHIND_ANKLE), toe + along * TIP_BEYOND_TOE)
}

/// How far, horizontally (world), to move a foot whose middle line runs
/// `heel` to `tip` so that no part of it is within `clearance` of an
/// obstacle; zero when it is clear.
pub trait FootObstacles: Send + Sync + 'static {
    fn clear(&self, heel: Vec3, tip: Vec3, clearance: f32) -> Vec3;
}

/// The obstacles a character's feet keep clear of
/// (`plugin::AnimFootIk`). Without one, the feet go where the animation
/// puts them.
#[derive(Component)]
pub struct AnimObstacles(pub Box<dyn FootObstacles>);

impl Default for AnimObstacles {
    /// None yet: filled in each frame by whatever owns it
    /// (`physics_obstacles`).
    fn default() -> Self {
        Self(Box::new(Footprints::default()))
    }
}

/// A box standing on the floor, seen from above: its middle, the way its
/// depth runs, and its size (width across, depth along).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Footprint {
    pub middle: Vec3,
    pub forward: Vec3,
    pub size: Vec2,
}

impl Footprint {
    fn axes(&self) -> (Vec3, Vec3) {
        let forward = Vec3::new(self.forward.x, 0.0, self.forward.z).normalize_or_zero();
        (Vec3::Y.cross(forward), forward)
    }

    /// `at` in the box's frame: (across, along) from its middle.
    pub fn local(&self, at: Vec3) -> Vec2 {
        let (left, forward) = self.axes();
        let off = at - self.middle;
        Vec2::new(off.dot(left), off.dot(forward))
    }

    /// A move in the box's frame, in the world.
    pub fn world_move(&self, local: Vec2) -> Vec3 {
        let (left, forward) = self.axes();
        left * local.x + forward * local.y
    }

    /// The move (box frame) taking the line `heel`-`tip` out to
    /// `clearance` from the box.
    ///
    /// Clear of the box, the line moves away from it along the line
    /// between their nearest points, so round its corners radially and
    /// continuously all the way round: out by the nearest side, a move
    /// flipped 20 cm from front to side at a corner's diagonal and the foot
    /// it moved swept past a chair's leg.
    ///
    /// Crossing it, the smallest move along the box's sides or square to
    /// the foot that separates them. As points instead of a line, a box
    /// under the foot's arch had the heel and toe pushed opposite ways, and
    /// the moves cancelled.
    fn out(&self, heel: Vec3, tip: Vec3, clearance: f32) -> Vec2 {
        let half = self.size * 0.5;
        let (a, b) = (self.local(heel), self.local(tip));
        let gap = |p: Vec2| p - p.clamp(-half, half);
        if !crosses(a, b, half) {
            // The nearest pair: an end of the line against the box, or a
            // corner of the box against the line.
            let along = b - a;
            let on_line = |p: Vec2| {
                let t = if along.length_squared() > 0.0 { ((p - a).dot(along) / along.length_squared()).clamp(0.0, 1.0) } else { 0.0 };
                a + along * t
            };
            let mut nearest = [a, b].map(gap).into_iter().min_by(|x, y| x.length().total_cmp(&y.length())).unwrap_or(Vec2::ZERO);
            for corner in [Vec2::new(1.0, 1.0), Vec2::new(1.0, -1.0), Vec2::new(-1.0, 1.0), Vec2::new(-1.0, -1.0)] {
                let point = on_line(corner * half);
                let away = gap(point);
                if away.length() < nearest.length() {
                    nearest = away;
                }
            }
            let distance = nearest.length();
            if distance >= clearance || distance == 0.0 {
                return Vec2::ZERO;
            }
            return nearest / distance * (clearance - distance);
        }
        // Separating moves along each candidate axis: how far the line must
        // go that way for its nearest end to be `clearance` past the box.
        let mut axes = vec![Vec2::X, Vec2::NEG_X, Vec2::Y, Vec2::NEG_Y];
        let along = (b - a).normalize_or_zero();
        if along != Vec2::ZERO {
            axes.extend([along.perp(), -along.perp()]);
        }
        axes.into_iter()
            .map(|axis| {
                let reach = half.x * axis.x.abs() + half.y * axis.y.abs();
                axis * (reach + clearance - a.dot(axis).min(b.dot(axis)))
            })
            .min_by(|x, y| x.length().total_cmp(&y.length()))
            .unwrap_or(Vec2::ZERO)
    }
}

/// Whether the line `a`-`b` passes through the box of half size `half`
/// about the origin.
fn crosses(a: Vec2, b: Vec2, half: Vec2) -> bool {
    let d = b - a;
    let (mut enter, mut leave) = (0.0_f32, 1.0_f32);
    for axis in 0..2 {
        if d[axis].abs() < 1.0e-9 {
            if a[axis].abs() > half[axis] {
                return false;
            }
            continue;
        }
        let (t0, t1) = ((-half[axis] - a[axis]) / d[axis], (half[axis] - a[axis]) / d[axis]);
        enter = enter.max(t0.min(t1));
        leave = leave.min(t0.max(t1));
    }
    enter <= leave
}

/// Boxes on the floor.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Footprints(pub Vec<Footprint>);

impl FootObstacles for Footprints {
    /// Several boxes' moves are combined each way along each world axis (the
    /// most any needs to go +x, and -x, and so on), so two on either side of
    /// a foot cancel sideways rather than one throwing it into the other.
    fn clear(&self, heel: Vec3, tip: Vec3, clearance: f32) -> Vec3 {
        let (mut most, mut least) = (Vec3::ZERO, Vec3::ZERO);
        for footprint in &self.0 {
            let out = footprint.world_move(footprint.out(heel, tip, clearance));
            most = most.max(out);
            least = least.min(out);
        }
        most + least
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A chair's footprint, 0.46 wide and 0.48 deep, facing +Z, its front
    /// edge at z = 0.24.
    fn chair() -> Footprints {
        Footprints(vec![Footprint { middle: Vec3::ZERO, forward: Vec3::Z, size: Vec2::new(0.46, 0.48) }])
    }

    #[test]
    fn a_foot_is_moved_just_out_of_the_box() {
        let chair = chair();
        let clearance = 0.05;
        let front = 0.24;
        let point = |z: f32| Vec3::new(0.0, 0.0, z);
        // Clear: no move.
        assert_eq!(chair.clear(point(front + 0.06), point(front + 0.2), clearance), Vec3::ZERO);
        // Pointing at it, the heel 3 cm under its front: out the front, to the
        // clearance, the way it is shortest.
        let moved = chair.clear(Vec3::new(0.1, 0.0, front - 0.03), Vec3::new(0.1, 0.0, front + 0.20), clearance);
        assert!((moved - Vec3::new(0.0, 0.0, 0.08)).length() < 1.0e-5, "{moved}");
        // Beside it, a little in: out the side.
        let moved = chair.clear(Vec3::new(0.26, 0.0, -0.1), Vec3::new(0.26, 0.0, 0.1), clearance);
        assert!((moved - Vec3::new(0.02, 0.0, 0.0)).length() < 1.0e-5, "{moved}");
        // Continuous as it crosses into the clearance.
        let at = |z: f32| chair.clear(point(z), point(z + 0.2), clearance).length();
        assert!(at(front + clearance + 0.001) == 0.0 && at(front + clearance - 0.001) < 0.0011);
    }

    #[test]
    fn a_post_under_the_arch_moves_the_foot_off_it_sideways() {
        // A table leg, 4 cm square, between a foot's heel and its toe: as
        // points, heel and toe pushed opposite ways and the moves cancelled.
        let leg = Footprints(vec![Footprint { middle: Vec3::ZERO, forward: Vec3::Z, size: Vec2::splat(0.04) }]);
        let moved = leg.clear(Vec3::new(0.01, 0.0, 0.12), Vec3::new(0.01, 0.0, -0.18), 0.08);
        // Square to the foot (x), off the leg's nearer side, the foot's line
        // 8 cm past it: 0.02 + 0.08 - 0.01 = 0.09.
        assert!((moved - Vec3::new(0.09, 0.0, 0.0)).length() < 1.0e-5, "{moved}");
    }

    #[test]
    fn going_round_a_corner_the_move_never_jumps() {
        // Out by the nearest side, the move flipped from front to side at the
        // diagonal.
        let chair = chair();
        let clearance = 0.05;
        let corner = Vec3::new(0.23, 0.0, 0.24);
        let mut previous: Option<Vec3> = None;
        for step in 0..=90 {
            let angle = (step as f32).to_radians();
            let out = Vec3::new(angle.sin(), 0.0, angle.cos());
            let point = corner + out * 0.03;
            let moved = chair.clear(point, point + out * 0.2, clearance);
            assert!((moved.length() - 0.02).abs() < 1.0e-4, "{step}°: {moved}");
            if let Some(previous) = previous {
                assert!(moved.distance(previous) < 0.001, "{step}°: jumped {:.4} m", moved.distance(previous));
            }
            previous = Some(moved);
        }
        // A whole foot sliding along the front and round the corner.
        let mut previous: Option<Vec3> = None;
        for step in 0..=120 {
            let x = 0.10 + step as f32 * 0.002;
            let moved = chair.clear(Vec3::new(x, 0.0, 0.26), Vec3::new(x + 0.12, 0.0, 0.34), clearance);
            if let Some(previous) = previous {
                assert!(moved.distance(previous) < 0.005, "x {x:.3}: jumped {:.4} m", moved.distance(previous));
            }
            previous = Some(moved);
        }
    }

    #[test]
    fn between_two_boxes_their_sideways_moves_cancel() {
        // Two table legs 0.2 apart, a foot between them near both: the two
        // sideways moves cancel rather than one winning and throwing it into
        // the other.
        let legs = Footprints(vec![
            Footprint { middle: Vec3::new(-0.1, 0.0, 0.0), forward: Vec3::Z, size: Vec2::splat(0.04) },
            Footprint { middle: Vec3::new(0.1, 0.0, 0.0), forward: Vec3::Z, size: Vec2::splat(0.04) },
        ]);
        let moved = legs.clear(Vec3::new(0.0, 0.0, -0.1), Vec3::new(0.0, 0.0, 0.1), 0.09);
        assert!(moved.length() < 1.0e-5, "{moved}");
    }
}
