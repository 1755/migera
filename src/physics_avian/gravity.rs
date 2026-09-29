//! Point-source gravity for avian3d bodies — avian3d's own built-in
//! `Gravity` resource is a single global uniform vector (flat "down"), with
//! no point-source/non-uniform mode. This project's own demoted custom
//! engine had one (`physics::solve_static::PhysicsGravity`, used by the
//! orbit-around-a-planet milestone), so this module reproduces it as a
//! deliberately small system built on avian3d's own `ConstantLinearAcceleration`
//! component — a persistent, per-body component avian3d itself folds into
//! its own solver every physics step with no special scheduling required
//! (confirmed via avian3d's own docs: unlike a one-shot `Forces`-based
//! impulse, this component's value is read automatically every step, so an
//! ordinary `Update`-schedule system that just mutates the component's
//! current value, recomputed from each body's current position, is
//! sufficient — no need to hook into avian3d's own `PhysicsSchedule`/
//! `SubstepSchedule` at all).

use avian3d::prelude::ConstantLinearAcceleration;
use bevy::prelude::*;

/// A point-source gravity center: every entity with a `ConstantLinearAcceleration`
/// component gets pulled toward `center` at a constant `magnitude` (not
/// inverse-square — matches the demoted custom engine's own
/// `PhysicsGravity::acceleration_at` convention exactly, see that type's own
/// doc comment for why constant-magnitude is the deliberate choice: it
/// keeps a small scene's gravity indistinguishable from flat "down" gravity
/// near the surface, while still curving correctly at a small planetary
/// body's scale for an orbit milestone).
#[derive(Resource, Clone, Copy, Debug)]
pub struct PointGravity {
    pub center: Vec3,
    pub magnitude: f32,
}

impl PointGravity {
    pub fn acceleration_at(&self, position: Vec3) -> Vec3 {
        (self.center - position).normalize_or_zero() * self.magnitude
    }
}

/// `Update`-schedule system: recomputes every gravity-affected body's
/// `ConstantLinearAcceleration` from its CURRENT position every frame (not
/// just once at spawn) — a point-source field's direction genuinely
/// changes as a body moves relative to the center, most visible for a body
/// orbiting or resting on a small planetary body. Bodies without
/// `ConstantLinearAcceleration` (e.g. an avian3d `Kinematic`/`Static` body,
/// which the component would be a no-op on anyway) are simply not matched
/// by this query.
pub fn apply_point_gravity(gravity: Res<PointGravity>, mut bodies: Query<(&Transform, &mut ConstantLinearAcceleration)>) {
    for (transform, mut acceleration) in &mut bodies {
        acceleration.0 = gravity.acceleration_at(transform.translation);
    }
}

/// Registers `PointGravity`'s own per-frame update system — deliberately a
/// tiny standalone plugin (not folded into a larger "physics_avian plugin"
/// that doesn't exist yet) since not every avian3d-based example needs
/// point-source gravity (most use avian3d's own flat `Gravity` resource
/// directly, no custom system needed at all).
pub struct PointGravityPlugin;

impl Plugin for PointGravityPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, apply_point_gravity);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn acceleration_points_from_body_toward_center() {
        let gravity = PointGravity { center: Vec3::ZERO, magnitude: 9.81 };
        let a = gravity.acceleration_at(Vec3::new(10.0, 0.0, 0.0));
        assert!((a - Vec3::new(-9.81, 0.0, 0.0)).length() < 1e-4, "expected pull toward center along -X, got {a:?}");
    }

    #[test]
    fn acceleration_at_the_center_itself_is_zero_not_nan() {
        let gravity = PointGravity { center: Vec3::ZERO, magnitude: 9.81 };
        let a = gravity.acceleration_at(Vec3::ZERO);
        assert_eq!(a, Vec3::ZERO, "expected zero acceleration exactly at the center, not NaN from normalizing a zero vector");
    }

    #[test]
    fn magnitude_does_not_change_with_distance() {
        // Deliberately NOT inverse-square -- matches the demoted custom
        // engine's own PhysicsGravity convention, see this module's own
        // doc comment for why.
        let gravity = PointGravity { center: Vec3::ZERO, magnitude: 9.81 };
        let near = gravity.acceleration_at(Vec3::new(1.0, 0.0, 0.0)).length();
        let far = gravity.acceleration_at(Vec3::new(1000.0, 0.0, 0.0)).length();
        assert!((near - far).abs() < 1e-3, "expected constant magnitude regardless of distance, got near={near} far={far}");
    }
}
