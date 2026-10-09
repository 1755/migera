//! Moving platforms: step 18 of the parkour steps beyond the first ten
//! (slides and long falls), last part. A walker stands, walks and jumps on
//! a moving top, and jumps or walks off it carrying its velocity.
//!
//! A walker on a platform moves in the platform's frame: each frame its
//! root (and a fall landed on it) is carried by however far the platform
//! moved ([`Platform::moved`]), its own motion unchanged on top. A jump
//! taken on it stays in its frame. A fall begun on it lands in its frame
//! if it comes down on it again, and otherwise falls in the world with the
//! platform's velocity added ([`frame_for_fall`]); a fall begun anywhere
//! that comes down on a platform is planned in that platform's frame, its
//! velocity relative to it. The hips' velocity is the same either way at
//! the hand-off: nothing jumps.
//!
//! The app owns where its platforms are: it writes them each frame through
//! a [`Platforms`] handle shared by the walker (`Walker::platforms`) and
//! the character's ground ([`PlatformGround`], so the feet stand on them).

use std::sync::{Arc, RwLock};

use bevy::math::{Vec2, Vec3};

use super::Falling;
use crate::character::anim::ground::{GroundHit, GroundProbe};

/// How far below a platform's top a point still stands on it, metres (a
/// foot reaching for it), as a ledge's top.
const TOP_BELOW: f32 = 0.3;
/// A root this near a platform's top, metres, stands on it.
const STOOD: f32 = 0.1;

/// A moving block's top: level, `length` along `way`, `width` across it,
/// `thick` deep under its top. Where it is now, how fast it moves, and how
/// far it moved since the frame before (the app's: what carries a walker
/// on it).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Platform {
    /// Its top's middle (the world).
    pub top: Vec3,
    /// Along its length (level, unit).
    pub way: Vec3,
    pub length: f32,
    pub width: f32,
    pub thick: f32,
    /// Its velocity now, m/s, and how far it moved over the last frame.
    pub velocity: Vec3,
    pub moved: Vec3,
}

impl Platform {
    /// A platform still at `top`.
    pub fn new(top: Vec3, way: Vec3, length: f32, width: f32, thick: f32) -> Self {
        let way = way.with_y(0.0).normalize_or(Vec3::NEG_Z);
        Self { top, way, length, width, thick, velocity: Vec3::ZERO, moved: Vec3::ZERO }
    }

    /// The same, at `top` now, moving `velocity`, moved `moved` since the
    /// frame before.
    pub fn moving(self, top: Vec3, velocity: Vec3, moved: Vec3) -> Self {
        Self { top, velocity, moved, ..self }
    }

    /// Its footprint's half extents along and across it, metres.
    fn half(&self) -> Vec2 {
        Vec2::new(0.5 * self.length, 0.5 * self.width)
    }

    /// Whether `at` is over (or under) its top.
    pub fn covers(&self, at: Vec3) -> bool {
        let d = (at - self.top).with_y(0.0);
        let across = Vec3::Y.cross(self.way);
        let half = self.half();
        d.dot(self.way).abs() <= half.x && d.dot(across).abs() <= half.y
    }

    /// Its top's height under `at`: over it and no further than a foot's
    /// reach below it.
    pub fn top_under(&self, at: Vec3) -> Option<f32> {
        (self.covers(at) && at.y > self.top.y - TOP_BELOW).then_some(self.top.y)
    }

    /// Whether a root at `root` stands on it.
    pub fn stood_on_by(&self, root: Vec3) -> bool {
        self.covers(root) && (root.y - self.top.y).abs() < STOOD
    }

    /// Whether its block stands at `point` between `low` and `high`.
    fn blocks(&self, point: Vec3, low: f32, high: f32) -> bool {
        self.covers(point) && self.top.y > low && self.top.y - self.thick < high
    }
}

/// The app's platforms where they are now, shared between whoever moves
/// them (writing each frame) and what reads them: the walker
/// (`Walker::platforms`) and the character's ground ([`PlatformGround`]).
#[derive(Clone, Default)]
pub struct Platforms(Arc<RwLock<Vec<Platform>>>);

impl Platforms {
    /// Where they are now, how fast and how far they moved (each frame, in
    /// the same order: a walker on one keeps its index).
    pub fn set(&self, platforms: Vec<Platform>) {
        if let Ok(mut now) = self.0.write() {
            *now = platforms;
        }
    }

    /// Where they are now.
    pub fn now(&self) -> Vec<Platform> {
        self.0.read().map(|now| now.clone()).unwrap_or_default()
    }

    /// Whether there are none.
    pub fn is_empty(&self) -> bool {
        self.0.read().map_or(true, |now| now.is_empty())
    }
}

impl std::fmt::Debug for Platforms {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("Platforms").field(&self.now()).finish()
    }
}

/// The ground `under` with the platforms' tops on it, where they are now.
pub struct PlatformGround {
    pub under: Box<dyn GroundProbe>,
    pub platforms: Platforms,
}

impl GroundProbe for PlatformGround {
    fn sample(&self, at: Vec3) -> Option<GroundHit> {
        let under = self.under.sample(at);
        let top = self.platforms.now().iter().filter_map(|platform| platform.top_under(at)).reduce(f32::max);
        match (top, under) {
            (Some(top), Some(under)) if under.height > top => Some(under),
            (Some(top), _) => Some(GroundHit::flat(top)),
            (None, under) => under,
        }
    }

    fn blocks(&self, point: Vec3, low: f32, high: f32) -> bool {
        self.platforms.now().iter().any(|platform| platform.blocks(point, low, high)) || self.under.blocks(point, low, high)
    }
}

/// The ground `ground` finds at `at` with the platforms left out: under a
/// platform's top, what is under it.
pub fn without_platforms(platforms: &[Platform], ground: &dyn Fn(Vec3) -> Option<f32>, at: Vec3) -> Option<f32> {
    let height = ground(at)?;
    match platforms.iter().find(|platform| platform.top_under(at) == Some(height)) {
        Some(platform) => without_platforms(platforms, ground, at.with_y(platform.top.y - platform.thick - TOP_BELOW)),
        None => Some(height),
    }
}

/// The frame a fall just begun lands in, and the fall planned in it: the
/// platform it began on (`riding`, an index into `platforms`) if it comes
/// down on it again; else the world, the platform's velocity added and its
/// landing found on the ground with the platforms left out; or, begun in
/// the world, a platform it comes down on, its velocity relative to it.
/// `ground` finds the ground's height at a point with the platforms on it
/// where they are now (the character's `PlatformGround`). Every fall keeps
/// the hips' velocity in the world as the fall had it.
pub fn frame_for_fall(falling: Falling, riding: Option<usize>, platforms: &[Platform], ground: &dyn Fn(Vec3) -> Option<f32>) -> (Falling, Option<usize>) {
    let without = |at: Vec3| without_platforms(platforms, ground, at);
    // Comes down on `platform` in its frame (its velocity already off).
    let lands_on = |falling: &Falling, platform: &Platform| {
        let mut onto = falling.clone();
        onto.land_on(&|at| platform.top_under(at));
        (onto.ground() == platform.top.y).then_some(onto)
    };
    if let Some(platform) = riding.and_then(|i| platforms.get(i)) {
        if let Some(onto) = lands_on(&falling, platform) {
            return (onto, riding);
        }
        let mut off = falling;
        off.carry(platform.velocity);
        let below = without(off.root().with_y(off.root().y + 0.05)).unwrap_or(off.ground());
        off.fall_to(below.min(off.ground()));
        off.land_on(&without);
        return (off, None);
    }
    for (i, platform) in platforms.iter().enumerate() {
        let mut relative = falling.clone();
        relative.carry(-platform.velocity);
        if let Some(onto) = lands_on(&relative, platform) {
            return (onto, Some(i));
        }
    }
    let mut world = falling;
    world.land_on(&without);
    (world, None)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::rig::{forward_kinematics_on, BoneSet, LocalPose, RigGeometry};
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
    use crate::character::skeleton::Bone;
    use bevy::math::Quat;

    const DT: f32 = 1.0 / 60.0;

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        let stood = stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        (stood, rig)
    }

    /// A platform's top is stood on over it, from a foot's reach below it
    /// up; the ground with it on finds it, and with it left out finds what
    /// is under it.
    #[test]
    fn a_platforms_top_is_stood_on_over_it() {
        let platform = Platform::new(Vec3::new(1.0, 1.0, -2.0), Vec3::X, 3.0, 2.0, 0.3);
        assert_eq!(platform.top_under(Vec3::new(2.4, 1.2, -1.1)), Some(1.0), "over a corner");
        assert_eq!(platform.top_under(Vec3::new(2.6, 1.2, -2.0)), None, "past its end");
        assert_eq!(platform.top_under(Vec3::new(1.0, 1.2, -0.9)), None, "past its side");
        assert_eq!(platform.top_under(Vec3::new(1.0, 0.5, -2.0)), None, "under it");
        assert!(platform.stood_on_by(Vec3::new(1.0, 1.05, -2.0)) && !platform.stood_on_by(Vec3::new(1.0, 1.3, -2.0)));
        let platforms = Platforms::default();
        platforms.set(vec![platform]);
        let ground = PlatformGround { under: Box::new(crate::character::anim::ground::FlatGround::default()), platforms: platforms.clone() };
        let height = |at: Vec3| ground.sample(at).map(|hit| hit.height);
        assert_eq!(height(Vec3::new(1.0, 1.5, -2.0)), Some(1.0));
        assert_eq!(height(Vec3::new(4.0, 1.5, -2.0)), Some(0.0));
        assert_eq!(without_platforms(&platforms.now(), &height, Vec3::new(1.0, 1.5, -2.0)), Some(0.0));
        assert!(ground.blocks(Vec3::new(1.0, 0.0, -2.0), 0.0, 1.8) && !ground.blocks(Vec3::new(1.0, 0.0, -2.0), 0.0, 0.6), "a 0.3 m slab 0.7 m up passed under");
    }

    /// The world positions of every joint of `falling` now, its root moved
    /// on by `carried`.
    fn joints(falling: &Falling, carried: Vec3, rig: &RigGeometry) -> BoneSet<Vec3> {
        let at = forward_kinematics_on(&falling.pose(rig), rig);
        let turn = Quat::from_rotation_y(falling.facing());
        BoneSet::from_fn(|bone| falling.root() + carried + turn * at[bone])
    }

    /// Walking off a platform's end as it moves, falling in the world with
    /// its velocity added, or landing back on it in its frame; and dropping
    /// onto a platform going by, in its frame: the hips' velocity in the
    /// world the same as the fall's at the hand-off, the path ballistic in
    /// the world, the feet coming down on the platform where it is then
    /// (or on the floor past it), every joint's path smooth.
    #[test]
    fn falls_carry_a_platforms_velocity_and_land_on_it_where_it_is() {
        let (stood, rig) = real_stood();
        let mut faults = Vec::new();
        let flat = |_: Vec3| Some(0.0f32);
        // (name, the platform's top, its velocity, the fall's root, its
        // velocity, the ground below it, riding, lands on the platform)
        let cases = [
            ("off its end, carried on", Vec3::new(0.0, 1.0, -1.5), Vec3::new(1.5, 0.0, 0.0), Vec3::new(0.0, 1.0, -3.0), Vec3::new(0.0, 0.0, -1.4), 0.0, true, false),
            ("off its side, carried along", Vec3::new(0.0, 1.0, -1.5), Vec3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 1.0, -0.45), Vec3::new(0.0, 0.0, 1.4), 0.0, true, false),
            ("dropped onto it going by", Vec3::new(-1.2, 1.0, -2.0), Vec3::new(1.5, 0.0, 0.0), Vec3::new(0.0, 2.5, -2.0), Vec3::ZERO, 0.0, false, true),
            ("a hop back down onto it", Vec3::new(0.0, 1.0, -1.5), Vec3::new(1.5, 0.0, 0.0), Vec3::new(0.0, 1.0, -1.5), Vec3::new(0.0, 3.0, -0.5), 0.0, true, true),
        ];
        for (name, top, velocity, root, own, below, riding, lands) in cases {
            let mut platform = Platform::new(top, Vec3::X, 3.0, 2.0, 0.3);
            platform.velocity = velocity;
            let ground = |at: Vec3| platform.top_under(at).or(flat(at));
            let falling = Falling::off(root, 0.0, own, &stood, below, 0.0, &stood, &rig);
            // The hips' world velocity as the walker had it: its own, plus
            // the platform's if it stood on it.
            let before = falling.hips_velocity() + if riding { velocity } else { Vec3::ZERO };
            let (mut planned, frame) = frame_for_fall(falling, riding.then_some(0), &[platform], &ground);
            let in_frame = frame.is_some();
            if in_frame != lands {
                faults.push(format!("{name}: in its frame {in_frame}, should be {lands}"));
                continue;
            }
            let carried_at = |t: f32| if in_frame { velocity * t } else { Vec3::ZERO };
            // The same fall made in the frame it falls in, as a plain fall:
            // what the hand-off is measured against.
            let own_in_frame = own + if riding { velocity } else { Vec3::ZERO } - if in_frame { velocity } else { Vec3::ZERO };
            let mut plain = Falling::off(root, 0.0, own_in_frame, &stood, below, 0.0, &stood, &rig);
            if in_frame {
                plain.land_on(&|at| platform.top_under(at));
            }
            let own_kink = {
                let (mut frames, mut kink): (Vec<BoneSet<Vec3>>, f32) = (Vec::new(), 0.0);
                while !plain.is_done() && frames.len() < 240 {
                    let now = joints(&plain, Vec3::ZERO, &rig);
                    if frames.len() >= 2 {
                        let n = frames.len();
                        kink = Bone::ALL.iter().map(|&b| (now[b] - 2.0 * frames[n - 1][b] + frames[n - 2][b]).length()).fold(kink, f32::max);
                    }
                    frames.push(now);
                    plain.advance(DT);
                }
                kink
            };
            let after = planned.hips_velocity() + if in_frame { velocity } else { Vec3::ZERO };
            if (after - before).length() > 1.0e-3 {
                faults.push(format!("{name}: the hips' velocity {before} became {after}"));
            }
            let mut frames: Vec<BoneSet<Vec3>> = Vec::new();
            let (mut kink, mut kink_at) = (0.0f32, 0.0f32);
            let mut t = 0.0;
            while !planned.is_done() && t < 4.0 {
                let now = joints(&planned, carried_at(t), &rig);
                if frames.len() >= 2 {
                    let n = frames.len();
                    let step = Bone::ALL.iter().map(|&b| (now[b] - 2.0 * frames[n - 1][b] + frames[n - 2][b]).length()).fold(0.0, f32::max);
                    if step > kink {
                        (kink, kink_at) = (step, t);
                    }
                }
                frames.push(now);
                planned.advance(DT);
                t += DT;
            }
            eprintln!("{name}: the largest change of step {kink:.4} at {kink_at:.3} (the plain fall's own {own_kink:.4})");
            // Where its feet came down: on the platform where it was then.
            let touch = planned.ends()[0];
            let platform_then = Platform { top: top + velocity * touch, ..platform };
            let feet = planned.feet().map(|foot| foot + carried_at(touch));
            let on = feet.iter().all(|&foot| platform_then.covers(foot) && (foot.y - platform_then.top.y).abs() < 0.2);
            eprintln!("{name}: frame {frame:?}, touchdown {touch:.3}, feet {feet:?}, on it {on}, kink {kink:.4}");
            if on != lands {
                faults.push(format!("{name}: the feet on the platform {on}, should be {lands}"));
            }
            // Nothing over the fall's own: the frame moves steadily. (Its
            // own is the landing's, touching down moving across the ground.)
            if kink > own_kink + 1.0e-3 {
                faults.push(format!("{name}: a step changed {kink:.4}, the plain fall's own {own_kink:.4}"));
            }
        }
        assert!(faults.is_empty(), "{faults:#?}");
    }
}
