//! The level's geometry the parkour moves act on, as plain values in the
//! world: what a hand can hold and a foot stand on.

use bevy::math::Vec3;

/// A ledge: the top edge of a wall or block, level, that hands can hang
/// from.
///
/// The wall's face is the vertical plane through the edge, facing `out`;
/// it reaches `wall_below` down from the edge (to the floor for a wall,
/// less for a block's face). With no wall below to brace the feet on, a
/// hang is free.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ledge {
    /// The edge's two ends, at the same height.
    pub a: Vec3,
    pub b: Vec3,
    /// Out of the wall, toward whoever hangs from it: horizontal, unit.
    pub out: Vec3,
    /// How deep the top is behind the edge, metres.
    pub depth: f32,
    /// How far the wall's face reaches down below the edge, metres.
    pub wall_below: f32,
}

impl Ledge {
    /// The top edge of a wall standing on the floor at `foot` (the middle of
    /// its face's foot), its face toward `out`, `width` wide and `height`
    /// high, its top `depth` deep.
    pub fn wall(foot: Vec3, out: Vec3, width: f32, height: f32, depth: f32) -> Self {
        let out = Vec3::new(out.x, 0.0, out.z).normalize_or(Vec3::Z);
        let along = Vec3::Y.cross(out);
        let middle = foot + Vec3::Y * height;
        Self { a: middle - along * (0.5 * width), b: middle + along * (0.5 * width), out, depth, wall_below: height }
    }

    /// The edge's height.
    pub fn height(&self) -> f32 {
        self.a.y
    }

    /// Along the edge, from `a` to `b`: unit.
    pub fn along(&self) -> Vec3 {
        (self.b - self.a).normalize_or(Vec3::X)
    }

    /// The point on the edge nearest `point`, seen from above, no nearer an
    /// end than `margin`.
    pub fn nearest(&self, point: Vec3, margin: f32) -> Vec3 {
        let length = (self.b - self.a).length();
        let s = (point - self.a).dot(self.along()).clamp(margin.min(0.5 * length), (length - margin).max(0.5 * length));
        self.a + self.along() * s
    }

    /// The point on the edge nearest `point`, seen from above, no nearer its
    /// end `a` than `margins[0]` nor `b` than `margins[1]` (the middle if
    /// they overlap).
    pub fn nearest_within(&self, point: Vec3, margins: [f32; 2]) -> Vec3 {
        let length = (self.b - self.a).length();
        let (low, high) = (margins[0], length - margins[1]);
        let s = (point - self.a).dot(self.along());
        let s = if low <= high { s.clamp(low, high) } else { 0.5 * (low + high) };
        self.a + self.along() * s
    }

    /// How far `point` is out in front of the wall's face, metres (negative:
    /// inside the wall).
    pub fn out_of(&self, point: Vec3) -> f32 {
        (point - self.a).dot(self.out)
    }

    /// The four ledges round the top of a block standing on the floor, its
    /// front face's foot at `foot` facing `out`, `width` wide (along the
    /// front), `depth` deep, `height` high: front, then round its corners.
    pub fn block(foot: Vec3, out: Vec3, width: f32, depth: f32, height: f32) -> [Self; 4] {
        let out = Vec3::new(out.x, 0.0, out.z).normalize_or(Vec3::Z);
        let across = Vec3::Y.cross(out);
        let middle = foot - out * (0.5 * depth);
        [(out, width, depth), (across, depth, width), (-out, width, depth), (-across, depth, width)]
            .map(|(face, long, deep)| Self::wall(middle + face * (0.5 * deep), face, long, height, deep))
    }

    /// The ledge among `others` that carries this one's edge on round a
    /// corner at its end `end` (0: `a`, 1: `b`), turned to start there (its
    /// `a` the corner); and how far the face turns about `+Y` going onto
    /// it, radians. `None` if no other ledge meets it there at its height.
    pub fn joined(&self, end: usize, others: &[Ledge]) -> Option<(Ledge, f32)> {
        let corner = if end == 0 { self.a } else { self.b };
        others.iter().find_map(|other| {
            // Not near parallel (within 20°): ruled out at 60°, a 45° or 135°
            // corner was never found.
            if (other.height() - self.height()).abs() > CORNER_GAP || other.along().dot(self.along()).abs() > PARALLEL {
                return None;
            }
            let next = if (other.a - corner).length() < CORNER_GAP {
                *other
            } else if (other.b - corner).length() < CORNER_GAP {
                Ledge { a: other.b, b: other.a, ..*other }
            } else {
                return None;
            };
            let turn = self.out.cross(next.out).y.atan2(self.out.dot(next.out));
            Some((next, turn))
        })
    }

    /// The wall's footprint on the floor below, for a walk to go round:
    /// `None` if it is a slab overhead, reaching less than a walker's
    /// height down (whose way under it is clear).
    pub fn footprint(&self) -> Option<crate::character::anim::obstacles::Footprint> {
        if self.wall_below < self.height() - 1.8 {
            return None;
        }
        let middle = 0.5 * (self.a + self.b) - self.out * (0.5 * self.depth);
        Some(crate::character::anim::obstacles::Footprint {
            middle: bevy::math::Vec3::new(middle.x, 0.0, middle.z),
            forward: self.out,
            size: bevy::math::Vec2::new((self.b - self.a).length(), self.depth),
        })
    }
}

/// How near two ledges' ends must be to meet at a corner, metres.
const CORNER_GAP: f32 = 0.02;
/// Two edges nearer parallel than this (the cosine between them, 20°) meet
/// in a straight run, not a corner.
const PARALLEL: f32 = 0.94;

/// A ground with ledges' tops on it: each top where it is (from just below
/// it up), else the ground `under` it. For a walker that climbs up onto a
/// ledge to stand there (`hang::HangAsk::ClimbUp`).
pub struct LedgeGround {
    pub under: Box<dyn crate::character::anim::ground::GroundProbe>,
    pub ledges: Vec<Ledge>,
}

/// How far below a ledge's top a point still stands on it, metres: a foot
/// reaching for it.
const TOP_BELOW: f32 = 0.3;

impl crate::character::anim::ground::GroundProbe for LedgeGround {
    fn sample(&self, at: Vec3) -> Option<crate::character::anim::ground::GroundHit> {
        for ledge in &self.ledges {
            let along = (at - ledge.a).dot(ledge.along());
            let back = -ledge.out_of(at);
            if (0.0..=(ledge.b - ledge.a).length()).contains(&along) && (0.0..=ledge.depth).contains(&back) && at.y > ledge.height() - TOP_BELOW {
                return Some(crate::character::anim::ground::GroundHit { height: ledge.height(), normal: Vec3::Y });
            }
        }
        self.under.sample(at)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A block's four ledges meet end to end, each turning a right angle
    /// outward (left about `+Y`, going round anticlockwise seen from above)
    /// onto the next; an inside corner turns the other way.
    #[test]
    fn a_blocks_ledges_meet_at_its_corners() {
        let ledges = Ledge::block(Vec3::new(0.0, 0.0, -0.5), Vec3::Z, 1.6, 1.0, 2.0);
        assert!((ledges[0].out - Vec3::Z).length() < 1.0e-6 && (ledges[0].a.z + 0.5).abs() < 1.0e-6);
        for k in 0..4 {
            let (next, turn) = ledges[k].joined(1, &ledges).unwrap_or_else(|| panic!("ledge {k}: no corner at its end"));
            assert!((next.a - ledges[k].b).length() < 1.0e-5, "ledge {k}: the next starts {:?} from its end", next.a - ledges[k].b);
            assert!((next.out - ledges[(k + 1) % 4].out).length() < 1.0e-5, "ledge {k}: carried onto the wrong face");
            assert!((turn - std::f32::consts::FRAC_PI_2).abs() < 1.0e-5, "ledge {k}: turns {turn} round an outside corner");
            let (back, turn) = ledges[k].joined(0, &ledges).expect("a corner at its start");
            assert!((back.out - ledges[(k + 3) % 4].out).length() < 1.0e-5 && (turn + std::f32::consts::FRAC_PI_2).abs() < 1.0e-5);
        }
        // An inside corner: a wall at the front's end facing back along it.
        let side = Ledge::wall(Vec3::new(0.8, 0.0, 0.0), Vec3::NEG_X, 1.0, 2.0, 0.5);
        let (next, turn) = ledges[0].joined(1, &[side]).expect("an inside corner");
        assert!((next.a - ledges[0].b).length() < 1.0e-5 && (turn + std::f32::consts::FRAC_PI_2).abs() < 1.0e-5, "turns {turn}");
        assert!(ledges[0].joined(1, &[Ledge { a: side.a + Vec3::Y, b: side.b + Vec3::Y, ..side }]).is_none(), "a ledge a metre higher is no corner");
    }

    /// On a ledge's top it stands at its height; in front of it, or below
    /// it, on the ground under it.
    #[test]
    fn a_ledges_top_is_stood_on_from_just_below_it_up() {
        use crate::character::anim::ground::{FlatGround, GroundProbe};
        let ledge = Ledge::wall(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 2.0, 2.0, 1.0);
        let ground = LedgeGround { under: Box::new(FlatGround::default()), ledges: vec![ledge] };
        let height = |at: Vec3| ground.sample(at).map(|hit| hit.height);
        assert_eq!(height(Vec3::new(0.2, 2.1, -1.4)), Some(2.0), "on the top");
        assert_eq!(height(Vec3::new(0.2, 1.8, -1.4)), Some(2.0), "a foot reaching for the top");
        assert_eq!(height(Vec3::new(0.2, 2.1, -0.6)), Some(0.0), "in front of the face");
        assert_eq!(height(Vec3::new(0.2, 0.5, -1.4)), Some(0.0), "well below the top");
        assert_eq!(height(Vec3::new(1.5, 2.1, -1.4)), Some(0.0), "past its end");
    }

    #[test]
    fn a_walls_edge_runs_along_its_top_square_to_its_face() {
        let ledge = Ledge::wall(Vec3::new(1.0, 0.0, -2.0), Vec3::Z, 2.0, 2.3, 1.0);
        assert!((ledge.height() - 2.3).abs() < 1.0e-6 && (ledge.b.y - 2.3).abs() < 1.0e-6);
        assert!(ledge.along().dot(ledge.out).abs() < 1.0e-6 && ((ledge.b - ledge.a).length() - 2.0).abs() < 1.0e-5);
        // A point in front of the face, beside the edge's middle.
        let point = Vec3::new(1.3, 1.0, -1.5);
        assert!((ledge.out_of(point) - 0.5).abs() < 1.0e-5);
        let near = ledge.nearest(point, 0.2);
        assert!((near - Vec3::new(1.3, 2.3, -2.0)).length() < 1.0e-5, "{near:?}");
        // Past an end, held `margin` in from it.
        assert!((ledge.nearest(Vec3::new(5.0, 0.0, -1.0), 0.2) - Vec3::new(1.8, 2.3, -2.0)).length() < 1.0e-5);
    }
}
