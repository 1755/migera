//! Ground from the physics world: feet that stand on what lies under them.
//!
//! The animation asks its [`GroundProbe`] for the ground under a point from
//! inside the IK, where there is no access to the physics world. So
//! [`sample_physics_ground`] raycasts beforehand, each frame, a small grid
//! under each foot (where the feet were last frame; they move a few
//! centimetres a frame), and hands the hits to a [`SampledGround`], which
//! answers the IK from them. Away from the feet it answers the floor, so the
//! body keeps to the floor while a foot plants on a prop.

use avian3d::prelude::{SpatialQuery, SpatialQueryFilter};
use bevy::prelude::*;

use super::ground::{GroundHit, GroundProbe};
use super::plugin::{AnimGround, AnimSet};
use super::{Ragdoll, WalkerSet};
use crate::character::{Bone, HumanoidSkeleton};

/// Samples the physics world under every [`PhysicsGround`] character's feet.
/// Needs avian's `PhysicsPlugins`.
pub struct PhysicsGroundPlugin;

impl Plugin for PhysicsGroundPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, sample_physics_ground.after(WalkerSet::Ride).before(AnimSet::Ik));
    }
}

/// Plant this character's feet on the physics world, not a fixed plane.
#[derive(Component, Debug, Clone)]
pub struct PhysicsGround {
    /// The floor's height, metres: the answer away from the feet, and below
    /// any hit lower than it.
    pub floor: f32,
    /// The highest surface above the ground the body stands on (`support`)
    /// a foot will stand on, metres; anything taller is an obstacle the foot
    /// does not climb. Measured from the floor, it hid every stair above the
    /// second and every ramp above 0.35 m.
    pub max_step: f32,
    /// The grid's spacing, metres, and how many samples it reaches out from
    /// the foot's centre each way.
    pub spacing: f32,
    pub reach: i32,
    /// Colliders the rays pass through besides the character's own ragdoll
    /// bodies (its kinematic proxy, say).
    pub ignore: Vec<Entity>,
    /// The ground the body stands on: the mean height under its two feet,
    /// eased over [`SUPPORT_SECONDS`]. Written by [`sample_physics_ground`].
    pub support: f32,
    /// That mean as sampled this frame, before the ease: well below
    /// `support`, the feet are over a drop.
    pub under: f32,
}

/// How quickly the body rises or sinks to the ground under its feet,
/// seconds (an exponential ease's time constant): a step up onto a
/// platform lifts it within a stride, without a pop. At 0.15 s the body
/// still crouched with both feet already up, the hips 6 cm short 0.2 s on.
pub const SUPPORT_SECONDS: f32 = 0.1;

impl Default for PhysicsGround {
    fn default() -> Self {
        // ±0.2 m about the ankle at 5 cm: the sole's 0.22 m ahead and 0.09
        // behind, with a margin for a foot moving a frame's worth.
        Self { floor: 0.0, max_step: 0.35, spacing: 0.05, reach: 4, ignore: Vec::new(), support: 0.0, under: 0.0 }
    }
}

/// How near the character's origin a query is the body's own, metres.
pub const BODY_REACH: f32 = 0.06;

/// Ground answered from raycast samples, else a flat floor.
#[derive(Debug, Clone, Default)]
pub struct SampledGround {
    pub floor: f32,
    /// Each sample's horizontal position (`x`, `z`) and what was hit there.
    pub samples: Vec<(Vec2, GroundHit)>,
    /// Their spacing, metres: a point further than this from every sample
    /// is answered by the floor.
    pub spacing: f32,
    /// The character's origin (`x`, `z`), answered by `support` however the
    /// grids overlap it: the walker's height follows the ground there, so the
    /// body stands on what its feet stand on. Answered by the floor instead,
    /// the pelvis stayed down while the feet walked onto a platform, the
    /// legs bent under it.
    pub body: Option<Vec2>,
    /// The height the body stands on, metres ([`PhysicsGround::support`]).
    pub support: f32,
}

impl GroundProbe for SampledGround {
    fn sample(&self, world_position: Vec3) -> Option<GroundHit> {
        let at = Vec2::new(world_position.x, world_position.z);
        // Within a frame's travel of it (the walker asks at its new
        // position, the probe was built at the last); the feet stand 11 cm
        // to either side.
        if self.body.is_some_and(|body| body.distance(at) < BODY_REACH) {
            return Some(GroundHit::flat(self.support));
        }
        // The highest sample within a spacing: a foot over a prop's edge
        // stands on it rather than flickering between it and the floor as
        // the nearest sample changes.
        let near = self.samples.iter().filter(|(p, _)| p.distance_squared(at) <= self.spacing * self.spacing).map(|(_, hit)| *hit);
        Some(near.max_by(|a, b| a.height.total_cmp(&b.height)).unwrap_or(GroundHit::flat(self.floor)))
    }
}

/// Raycasts each [`PhysicsGround`] character's grids and replaces its
/// [`AnimGround`] with what they found.
pub fn sample_physics_ground(
    time: Res<Time>,
    spatial: SpatialQuery,
    mut characters: Query<(&HumanoidSkeleton, &mut PhysicsGround, &mut AnimGround, &Transform, Option<&Ragdoll>)>,
    world_transforms: Query<&GlobalTransform>,
) {
    for (skeleton, mut config, mut ground, root, ragdoll) in &mut characters {
        let mut ignore: Vec<Entity> = config.ignore.clone();
        if let Some(ragdoll) = ragdoll {
            ignore.extend(ragdoll.bodies.iter().filter_map(|(_, body)| *body));
        }
        let filter = SpatialQueryFilter::default().with_excluded_entities(ignore);
        let mut samples = Vec::new();
        let mut soles = Vec::new();
        for (ankle, toe) in [(Bone::LeftFoot, Bone::LeftToeBase), (Bone::RightFoot, Bone::RightToeBase)] {
            let (Ok(ankle), Ok(toe)) = (world_transforms.get(skeleton.entity(ankle)), world_transforms.get(skeleton.entity(toe))) else { continue };
            // Centred between ankle and ball, the sole's middle.
            let centre = (ankle.translation() + toe.translation()) * 0.5;
            soles.push(centre);
            // From a step above the ground the body stands on, down to the
            // floor: a stair or ramp the body is on, and any drop below it.
            let from = config.support + config.max_step + 0.3;
            for i in -config.reach..=config.reach {
                for j in -config.reach..=config.reach {
                    let x = centre.x + i as f32 * config.spacing;
                    let z = centre.z + j as f32 * config.spacing;
                    let Some(hit) = spatial.cast_ray(Vec3::new(x, from, z), Dir3::NEG_Y, from - config.floor + 0.05, true, &filter) else { continue };
                    let height = from - hit.distance;
                    // The floor itself, or anything too tall to step onto.
                    if height <= config.floor + 1.0e-3 || height > config.support + config.max_step {
                        continue;
                    }
                    samples.push((Vec2::new(x, z), GroundHit { height, normal: hit.normal }));
                }
            }
        }
        // The body's ground: the mean of what lies under each sole, eased.
        let mut sampled = SampledGround { floor: config.floor, samples, spacing: config.spacing, body: None, support: config.floor };
        if !soles.is_empty() {
            let under = soles.iter().map(|sole| sampled.sample(*sole).map_or(config.floor, |hit| hit.height)).sum::<f32>() / soles.len() as f32;
            config.under = under;
            let ease = 1.0 - (-time.delta_secs() / SUPPORT_SECONDS).exp();
            config.support += (under - config.support) * ease;
        }
        sampled.body = Some(Vec2::new(root.translation.x, root.translation.z));
        sampled.support = config.support;
        ground.0 = Box::new(sampled);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sampled_ground_stands_on_a_prop_the_floor_elsewhere_and_the_body_on_its_support() {
        let ground = SampledGround {
            floor: 0.0,
            samples: vec![(Vec2::new(1.0, 1.0), GroundHit::flat(0.2)), (Vec2::new(1.05, 1.0), GroundHit::flat(0.25))],
            spacing: 0.05,
            body: Some(Vec2::new(1.5, 1.0)),
            support: 0.1,
        };
        // On the prop, the higher of the samples within reach.
        assert_eq!(ground.sample(Vec3::new(1.03, 0.5, 1.0)).unwrap().height, 0.25);
        // Off it, the floor.
        assert_eq!(ground.sample(Vec3::new(2.0, 0.5, 1.0)).unwrap().height, 0.0);
        // At the character's origin, the ground its feet stand on.
        assert_eq!(ground.sample(Vec3::new(1.52, 0.0, 1.0)).unwrap().height, 0.1);
    }
}
