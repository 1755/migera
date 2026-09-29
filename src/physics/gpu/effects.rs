//! Visual-effects-only GPU physics body components — the data contract
//! `extract.rs` reads for the demoted custom engine's own GPU compute path,
//! now explicitly re-scoped away from "the GPU port of the primary
//! solver" (that role belongs to `avian3d`, see `crate::physics_avian`'s
//! own module doc comment) to large counts of simple, individually-
//! inconsequential bodies — debris, particle-like chunks, background
//! clutter — where GPU throughput is the actual goal and per-body
//! simulation correctness against gameplay-relevant bodies is explicitly
//! NOT a goal.
//!
//! Deliberately does NOT reuse `physics::components::RigidBody`/`Inertia`/
//! `BodyKind` — those are the demoted CPU engine's own solver-facing types,
//! and keeping the GPU-effects path's own component vocabulary named after
//! them would be confusing now that no CPU solver consumes this data at
//! all. `BodyKind` specifically has no debris equivalent: every debris
//! body IS the equivalent of today's `BodyKind::Dynamic` (a real,
//! gravity/contact-affected body) — kinematic platforms are an
//! interactive/gameplay feature now owned by `avian3d`, and a debris chunk
//! has no reason to ever be "kinematic." `physics::components::PhysicsShape`
//! IS still reused here, deliberately — it's a plain collider-shape enum
//! shared across every physics-relevant subsystem in this project (not
//! solver-specific the way `RigidBody`/`Inertia`/`BodyKind` are), and
//! `PhysicsShapeGpu::from_shape`/`bounding_radius` (both already existing,
//! unchanged) already consume it directly.

use bevy::prelude::*;

/// A GPU-effects debris body's linear/angular velocity and mass — the
/// visual-effects-only equivalent of `physics::components::RigidBody` +
/// `physics::components::Inertia` combined into one component, since a
/// debris body always needs both together (nothing in this path ever
/// wants velocity without mass or vice versa, unlike the demoted engine's
/// own static/kinematic bodies which need zero-mass-with-real-velocity or
/// zero-mass-with-zero-velocity combinations `Inertia`/`RigidBody` being
/// separate components supported).
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct GpuDebrisBody {
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub inverse_mass: f32,
    /// Diagonal of the body-local inverse inertia tensor — same convention
    /// `physics::components::Inertia::inverse_tensor_diag` already uses.
    pub inverse_inertia_local: Vec3,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn debris_body_fields_are_independently_settable() {
        let body = GpuDebrisBody { linear_velocity: Vec3::new(1.0, 2.0, 3.0), angular_velocity: Vec3::ZERO, inverse_mass: 0.5, inverse_inertia_local: Vec3::splat(0.1) };
        assert_eq!(body.linear_velocity, Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(body.inverse_mass, 0.5);
    }
}
