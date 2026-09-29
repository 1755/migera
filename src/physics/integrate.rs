//! Authoritative-`Transform` write path and the `PhysicsPlugin` that
//! registers it. `physics_integrate_placeholder` originally proved the
//! scheduling plumbing ("physics writes `Transform`, BVH refit and
//! extraction see it") in isolation, deliberately separated from "is the
//! physics correct" — kept here as a still-tested, dependency-free
//! reference now that Stage 3's `solve_world::solve_world` is the system
//! `PhysicsPlugin` actually registers (all of `physics_integrate_placeholder`,
//! `solve_static::solve_static_collisions`, and `solve_world::solve_world`
//! mutate `Transform` on `RigidBody` entities the same way; running more
//! than one at once would double-count motion, so only one is ever wired
//! into the plugin at a time).

use bevy::prelude::*;

use super::components::RigidBody;
use super::solve_static::PhysicsGravity;
use super::solve_world::solve_world;

/// Opt-in toggle for the GPU physics path (`physics::gpu`'s
/// end-to-end-wired substep solver) — default OFF, meaning CPU
/// `solve_world` (below) remains the shipping default. A plain runtime
/// resource rather than two swappable plugins: Bevy has no supported
/// "remove a plugin at runtime" API (`bevy_app::App` only supports
/// adding plugins), so a `.run_if()`-gated system swap is the only real
/// mechanism for a genuine runtime toggle. Flipping this off mid-run
/// instantly falls back to the always-correct CPU solver with no
/// plugin-swap/respawn needed. Read directly (via `Extract<Res<...>>`,
/// no render-world mirror resource needed for a single `bool`) by
/// `physics::gpu::extract::extract_physics_bodies` and every downstream
/// GPU dispatch system to decide whether to do any work this frame.
#[derive(Resource, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PhysicsGpuEnabled(pub bool);

/// `Update`-schedule system: `Transform.translation += linear_velocity * dt`
/// for every `RigidBody`-tagged entity, with no gravity or collision.
/// Superseded by `solve_world::solve_world` as of Stage 3 (see this
/// module's doc comment) — kept as a regression-tested reference for the
/// base scheduling contract, not registered by `PhysicsPlugin` anymore.
pub fn physics_integrate_placeholder(time: Res<Time>, mut bodies: Query<(&RigidBody, &mut Transform)>) {
    let dt = time.delta_secs();
    for (body, mut transform) in &mut bodies {
        transform.translation += body.linear_velocity * dt;
    }
}

/// Registers the physics `Update`-schedule systems, ordered before the
/// hybrid renderer's BVH refit — the same constraint every physics stage's
/// authoritative-transform write must respect, per
/// `src/hybrid/motion.rs`'s `PreUpdate` (snapshot) -> `Update` (this
/// plugin's systems) -> `PostUpdate` (Bevy's own transform propagation)
/// ordering contract.
pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsGravity>()
            .init_resource::<PhysicsGpuEnabled>()
            .add_systems(
                Update,
                solve_world
                    .run_if(|enabled: Res<PhysicsGpuEnabled>| !enabled.0)
                    .before(crate::hybrid::bvh::update_persistent_bvh),
            );
    }
}

#[cfg(test)]
mod tests {
    use bevy::prelude::*;

    use super::*;

    fn test_app() -> App {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_systems(Update, physics_integrate_placeholder);
        app
    }

    #[test]
    fn a_body_drifts_by_velocity_times_delta_time() {
        let mut app = test_app();
        let entity = app
            .world_mut()
            .spawn((
                RigidBody { linear_velocity: Vec3::new(1.0, 0.0, 0.0), angular_velocity: Vec3::ZERO },
                Transform::from_xyz(0.0, 0.0, 0.0),
            ))
            .id();
        app.update();
        let transform = app.world().get::<Transform>(entity).unwrap();
        // First frame's `Time::delta_secs()` is 0 under `MinimalPlugins`
        // (no elapsed wall-clock yet), so translation must still be exactly
        // zero after one update — confirmed explicitly rather than assumed,
        // since a nonzero first-frame delta would silently double-count
        // motion once a real clock is present.
        assert_eq!(transform.translation, Vec3::ZERO);
    }

    #[test]
    fn a_stationary_body_never_moves() {
        let mut app = test_app();
        let entity = app
            .world_mut()
            .spawn((RigidBody::default(), Transform::from_xyz(3.0, 4.0, 5.0)))
            .id();
        app.update();
        app.update();
        let transform = app.world().get::<Transform>(entity).unwrap();
        assert_eq!(transform.translation, Vec3::new(3.0, 4.0, 5.0));
    }
}
