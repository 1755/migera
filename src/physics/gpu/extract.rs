//! `ExtractSchedule` system snapshotting the main-world GPU-effects debris
//! body set into a render-world resource — the first foothold physics-
//! domain data has in the render world specifically for the GPU visual-
//! effects path. Deliberately a NEW file, not folded into `hybrid::extract`'s
//! own `extract_hybrid_scene`: this data has nothing to do with the
//! renderer's own `ObjectGpu`/BVH-node extraction, and this extraction has
//! an entirely separate gating condition (`PhysicsGpuEnabled`) that
//! `extract_hybrid_scene` doesn't share.
//!
//! Mirrors `extract_hybrid_scene`'s own `Extract<Query<...>>` aggregation
//! pattern (`hybrid/extract.rs:1029-1064`) exactly, including its
//! `object_order: Vec<Entity>` precedent: this module's own
//! `dynamic_entities` field is that same idea, applied here so
//! `physics::gpu::readback`'s one-frame-later apply system knows which
//! entity each GPU result index belongs to.
//!
//! **Re-scoped to visual-effects-only** (debris/particle-like bodies, see
//! `effects::GpuDebrisBody`'s own doc comment for the full reasoning): this
//! extraction now reads `GpuDebrisBody` instead of the demoted CPU engine's
//! own `RigidBody`/`Inertia`, and has dropped kinematic-body support
//! entirely (`BodyKind`/the dynamic-vs-kinematic contact-generation range)
//! — a debris chunk is always the equivalent of the old `BodyKind::Dynamic`,
//! and moving-platform-style interactive bodies are now `avian3d`'s own
//! responsibility (`crate::physics_avian`). Dynamic-vs-static collision is
//! KEPT (debris settling on a floor/terrain is exactly the showcase this
//! path exists for) — only the dynamic-vs-kinematic range was dropped.
//! Every GPU-side dispatch function/WGSL shader downstream of this file
//! needed NO changes: they only ever read `RenderPhysicsGpuFrame`'s own
//! already-GPU-typed fields (`PhysicsBodyGpu`/`PhysicsShapeGpu`), never the
//! CPU-side component types this file itself queries — the kinematic range
//! simply stays permanently empty (`kinematic_count` always `0`) as seen
//! by every downstream pass, the exact same code path already exercised
//! and tested by the pre-kinematic-support Piece 5 code and the
//! `kinematic_count == 0` regression test.

use bevy::prelude::*;
use bevy::render::Extract;

use super::super::components::PhysicsShape;
use super::super::integrate::PhysicsGpuEnabled;
use super::super::solve_static::{PhysicsGravity, bounding_radius};
use super::super::solve_world::SUBSTEPS;
use super::effects::GpuDebrisBody;
use super::types::{PhysicsBodyGpu, PhysicsShapeGpu};

/// Render-world snapshot of this frame's GPU-effects debris body set —
/// fully replaced every extract (no partial update), matching
/// `RenderHybridScene`'s own "fully replaced every extract" convention.
/// Body ordering is dynamics-then-statics (`0..dynamic_count` dynamics,
/// `dynamic_count..` statics) — `kinematic_count` is kept as a field
/// (rather than removed) purely because every downstream GPU dispatch
/// function/WGSL shader in this port already takes a `kinematic_count`
/// parameter (added for the now-demoted CPU engine's own kinematic-body
/// support) and expects the three-range convention; this extraction
/// simply always produces `kinematic_count == 0`, keeping every
/// downstream pass's own code and tests unchanged rather than threading a
/// two-range-vs-three-range distinction through the whole dispatch chain
/// for a range that will always be empty on this path.
#[derive(Resource, Default)]
pub struct RenderPhysicsGpuFrame {
    pub bodies: Vec<PhysicsBodyGpu>,
    pub shapes: Vec<PhysicsShapeGpu>,
    /// Dynamics only — statics never receive a `Transform` write-back from
    /// GPU readback, so their entities don't need tracking past this
    /// extract.
    pub dynamic_entities: Vec<Entity>,
    pub dynamic_count: u32,
    /// Always `0` on this path — see this struct's own doc comment for why
    /// the field is kept rather than removed.
    pub kinematic_count: u32,
    pub static_count: u32,
    pub substep_dt: f32,
    pub gravity_center: Vec3,
    pub gravity_magnitude: f32,
    /// Broad-phase cell size (~2x the largest dynamic body's own bounding
    /// radius) — same sizing convention `generate_all_contacts` itself
    /// uses, computed here (main-world data) since the render-world
    /// dispatch orchestration has no direct access to `PhysicsShape`.
    pub broadphase_cell_size: f32,
}

/// `ExtractSchedule` system: early-returns (clearing `out` to empty) when
/// `PhysicsGpuEnabled` is off or `dt <= 0.0`, mirroring `solve_world`'s
/// own early-return shape exactly (`solve_world.rs:67-70`). `SUBSTEPS` is
/// `solve_world`'s own constant (widened to `pub(crate)` for this reason)
/// — a single source of truth for substep count between the demoted CPU
/// engine and this GPU-effects path, not a second hand-copied value.
#[allow(clippy::type_complexity, clippy::too_many_arguments)]
pub fn extract_physics_bodies(
    enabled: Extract<Res<PhysicsGpuEnabled>>,
    time: Extract<Res<Time>>,
    gravity: Extract<Res<PhysicsGravity>>,
    statics: Extract<Query<(&PhysicsShape, &GlobalTransform), Without<GpuDebrisBody>>>,
    dynamics: Extract<Query<(Entity, &GpuDebrisBody, &PhysicsShape, &Transform)>>,
    mut out: ResMut<RenderPhysicsGpuFrame>,
) {
    if !enabled.0 {
        *out = RenderPhysicsGpuFrame::default();
        return;
    }
    let dt = time.delta_secs();
    if dt <= 0.0 {
        *out = RenderPhysicsGpuFrame::default();
        return;
    }

    let dynamic_count = dynamics.iter().count();
    let static_count = statics.iter().count();

    let mut bodies = Vec::with_capacity(dynamic_count + static_count);
    let mut shapes = Vec::with_capacity(dynamic_count + static_count);
    let mut dynamic_entities = Vec::with_capacity(dynamic_count);
    let mut max_dynamic_radius = 0.0f32;

    for (entity, debris_body, shape, transform) in &dynamics {
        bodies.push(PhysicsBodyGpu::from_state(transform.translation, transform.rotation, debris_body.linear_velocity, debris_body.angular_velocity, debris_body.inverse_mass, debris_body.inverse_inertia_local));
        shapes.push(PhysicsShapeGpu::from_shape(*shape));
        dynamic_entities.push(entity);
        max_dynamic_radius = max_dynamic_radius.max(bounding_radius(shape));
    }
    for (shape, transform) in &statics {
        bodies.push(PhysicsBodyGpu::from_state(transform.translation(), transform.rotation(), Vec3::ZERO, Vec3::ZERO, 0.0, Vec3::ZERO));
        shapes.push(PhysicsShapeGpu::from_shape(*shape));
    }

    out.bodies = bodies;
    out.shapes = shapes;
    out.dynamic_entities = dynamic_entities;
    out.dynamic_count = dynamic_count as u32;
    out.kinematic_count = 0;
    out.static_count = static_count as u32;
    out.substep_dt = dt / SUBSTEPS as f32;
    out.gravity_center = gravity.center;
    out.gravity_magnitude = gravity.magnitude;
    out.broadphase_cell_size = max_dynamic_radius.max(0.01) * 2.0;
}
