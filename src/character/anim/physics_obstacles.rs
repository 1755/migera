//! Obstacles from the physics world, for the feet and for the walk: chairs,
//! tables and their legs, walls, any collider marked [`Obstacle`].
//!
//! The foot IK asks its `obstacles::AnimObstacles` from inside the IK, where
//! there is no access to the physics world. So [`sample_physics_obstacles`]
//! asks beforehand, each frame: avian's broad phase for marked colliders
//! within [`PhysicsObstacles::reach`] of the feet (where they were last
//! frame) and reaching foot height above the ground the body stands on.
//! Each becomes a `obstacles::Footprint`: an upright box collider exactly
//! (turned about the vertical), anything else by its bounding box.
//!
//! And for a walk to a chair (`approach`), `obstacles::RouteObstacles`:
//! those around the character and its chair at body height (a table top
//! too), each leg dropped where it lies within its seat or top.
//!
//! # Opt-in, by marker
//!
//! Which colliders a foot keeps out of cannot be read off their shape: a
//! ramp is taller than a step yet walked up, a stair is a prop the foot
//! stands on (`physics_ground`), and both would push a foot off them as
//! obstacles. So only colliders marked [`Obstacle`] count: furniture,
//! walls, posts.

use avian3d::prelude::{Collider, ColliderAabb, SpatialQuery};
use bevy::prelude::*;

use super::obstacles::{AnimObstacles, Footprint, Footprints, RouteObstacles};
use super::physics_ground::PhysicsGround;
use super::plugin::AnimSet;
use super::{Ragdoll, Walker, WalkerSet};
use crate::character::{Bone, HumanoidSkeleton};

/// Gathers each [`PhysicsObstacles`] character's foot obstacles from the
/// physics world. Needs avian's `PhysicsPlugins`.
pub struct PhysicsObstaclesPlugin;

impl Plugin for PhysicsObstaclesPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, sample_physics_obstacles.after(WalkerSet::Ride).before(AnimSet::Ik));
    }
}

/// A collider feet keep out of (`obstacles`) and walks go round
/// (`approach`).
#[derive(Component, Debug, Clone, Copy, Default)]
pub struct Obstacle;

/// Keep this character's feet out of the physics world's [`Obstacle`]s,
/// and its walks to a chair round them (`obstacles::RouteObstacles`).
#[derive(Component, Debug, Clone)]
#[require(AnimObstacles, RouteObstacles)]
pub struct PhysicsObstacles {
    /// How far round the feet to look, metres: a swinging foot moves a
    /// stride's worth between where it was and where it lands, but only
    /// those within the foot's clearance move it.
    pub reach: f32,
    /// The height above the ground the body stands on that the feet reach,
    /// metres: an obstacle wholly above it (a table top) or below its
    /// bottom (a mat) is no obstacle to a foot.
    pub feet_height: (f32, f32),
    /// Colliders ignored besides the character's own ragdoll bodies.
    pub ignore: Vec<Entity>,
    /// How far round the character to look for what its walk goes round,
    /// metres, and the heights over its ground the body fills (a table top
    /// at 0.75 m is in the way of the body, not of the feet).
    pub route_reach: f32,
    pub body_height: (f32, f32),
    /// How many obstacles were found last frame, for the feet and the walk.
    pub found: usize,
    pub found_on_route: usize,
}

impl Default for PhysicsObstacles {
    fn default() -> Self {
        Self { reach: 0.5, feet_height: (0.01, 0.25), ignore: Vec::new(), route_reach: 2.5, body_height: (0.01, 1.8), found: 0, found_on_route: 0 }
    }
}

/// The footprint of a collider at `transform` with world bounds `aabb`:
/// an upright box exactly, turned about the vertical; anything else (or a
/// tilted box) by its bounds.
pub fn footprint_of(collider: &Collider, transform: &GlobalTransform, aabb: &ColliderAabb) -> Footprint {
    let (_, rotation, translation) = transform.to_scale_rotation_translation();
    let up = rotation * Vec3::Y;
    if let Some(cuboid) = collider.shape_scaled().as_cuboid()
        && up.y > 0.999
    {
        let half = cuboid.half_extents;
        return Footprint { middle: Vec3::new(translation.x, 0.0, translation.z), forward: rotation * Vec3::Z, size: Vec2::new(half.x, half.z) * 2.0 };
    }
    let (min, max) = (aabb.min, aabb.max);
    Footprint { middle: Vec3::new((min.x + max.x) * 0.5, 0.0, (min.z + max.z) * 0.5), forward: Vec3::Z, size: Vec2::new(max.x - min.x, max.z - min.z) }
}

/// Finds each [`PhysicsObstacles`] character's foot obstacles near its feet
/// and gives them to its [`AnimObstacles`].
#[allow(clippy::type_complexity)]
pub fn sample_physics_obstacles(
    spatial: SpatialQuery,
    mut characters: Query<(
        &HumanoidSkeleton,
        &mut PhysicsObstacles,
        &mut AnimObstacles,
        &mut RouteObstacles,
        &Transform,
        Option<&PhysicsGround>,
        Option<&Ragdoll>,
        Option<&Walker>,
    )>,
    obstacles: Query<(&Collider, &GlobalTransform, &ColliderAabb), With<Obstacle>>,
    world_transforms: Query<&GlobalTransform>,
) {
    for (skeleton, mut config, mut found, mut on_route, root, ground, ragdoll, walker) in &mut characters {
        let mut ignore = config.ignore.clone();
        if let Some(ragdoll) = ragdoll {
            ignore.extend(ragdoll.bodies.iter().filter_map(|(_, body)| *body));
        }
        let feet: Vec<Vec3> = [Bone::LeftFoot, Bone::LeftToeBase, Bone::RightFoot, Bone::RightToeBase]
            .into_iter()
            .filter_map(|bone| world_transforms.get(skeleton.entity(bone)).ok().map(GlobalTransform::translation))
            .collect();
        if feet.is_empty() {
            continue;
        }
        // The band of heights the feet sweep, over the ground the body
        // stands on.
        let floor = ground.map_or(root.translation.y, |ground| ground.support);
        let (low, high) = (floor + config.feet_height.0, floor + config.feet_height.1);
        let (min, max) = feet.iter().fold((Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(min, max), &p| (min.min(p), max.max(p)));
        let region = ColliderAabb { min: Vec3::new(min.x - config.reach, low, min.z - config.reach), max: Vec3::new(max.x + config.reach, high, max.z + config.reach) };
        let footprints: Vec<Footprint> = spatial
            .aabb_intersections_with_aabb(region)
            .into_iter()
            .filter(|entity| !ignore.contains(entity))
            .filter_map(|entity| obstacles.get(entity).ok())
            .filter(|(_, _, aabb)| aabb.min.y < high && aabb.max.y > low)
            .map(|(collider, transform, aabb)| footprint_of(collider, transform, aabb))
            .collect();
        config.found = footprints.len();
        found.0 = Box::new(Footprints(footprints));

        // What the walk goes round: around the character and the chair it
        // walks to, at body height. Around the character alone, the far
        // chairs at a table were found only on the way, after the walk had
        // chosen where to come at its own chair from.
        let (low, high) = (floor + config.body_height.0, floor + config.body_height.1);
        let at = root.translation;
        let to = walker.and_then(|walker| walker.chair).map_or(at, |chair| chair.seat);
        let region = ColliderAabb {
            min: Vec3::new(at.x.min(to.x) - config.route_reach, low, at.z.min(to.z) - config.route_reach),
            max: Vec3::new(at.x.max(to.x) + config.route_reach, high, at.z.max(to.z) + config.route_reach),
        };
        let route: Vec<Footprint> = spatial
            .aabb_intersections_with_aabb(region)
            .into_iter()
            .filter(|entity| !ignore.contains(entity))
            .filter_map(|entity| obstacles.get(entity).ok())
            .filter(|(_, _, aabb)| aabb.min.y < high && aabb.max.y > low)
            .map(|(collider, transform, aabb)| footprint_of(collider, transform, aabb))
            .collect();
        // Each piece is a collider: the legs lie within their seat or top,
        // and only add corners to the walk's graph.
        let route: Vec<Footprint> = route
            .iter()
            .enumerate()
            .filter(|&(i, piece)| !route.iter().enumerate().any(|(j, other)| j != i && other.size.x * other.size.y > piece.size.x * piece.size.y && other.contains(piece, 0.01)))
            .map(|(_, piece)| *piece)
            .collect();
        config.found_on_route = route.len();
        on_route.0 = route;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_upright_box_keeps_its_turn_and_anything_else_its_bounds() {
        // A table leg 4 by 6 cm, turned 30° about the vertical: its own
        // footprint, not the larger bounds.
        let leg = Collider::cuboid(0.04, 0.7, 0.06);
        let turn = Quat::from_rotation_y(30f32.to_radians());
        let transform = GlobalTransform::from(Transform::from_xyz(1.0, 0.35, 2.0).with_rotation(turn));
        let aabb = ColliderAabb { min: Vec3::new(0.9, 0.0, 1.9), max: Vec3::new(1.1, 0.7, 2.1) };
        let footprint = footprint_of(&leg, &transform, &aabb);
        assert!((footprint.size - Vec2::new(0.04, 0.06)).length() < 1.0e-5, "{:?}", footprint.size);
        assert!((footprint.forward - turn * Vec3::Z).length() < 1.0e-5);
        assert!((footprint.middle - Vec3::new(1.0, 0.0, 2.0)).length() < 1.0e-5);
        // A ball: its bounds.
        let ball = Collider::sphere(0.1);
        let footprint = footprint_of(&ball, &transform, &aabb);
        assert!((footprint.size - Vec2::new(0.2, 0.2)).length() < 1.0e-5 && footprint.forward == Vec3::Z);
    }
}
