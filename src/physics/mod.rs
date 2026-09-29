//! **Demoted from primary-engine role (2026-09).** This module is a
//! from-scratch, SDF-native rigid-body physics engine (CPU + GPU compute
//! port) built to collide directly against `sdf::components::Shape`
//! geometry (distance+gradient queries, no mesh/hull colliders) instead of
//! adopting a mature CPU engine like Rapier/avian3d. A later side-by-side
//! comparison found that justification — "free collision against
//! arbitrarily complex SDF/CSG geometry a mesh-based engine would need
//! convex decomposition for" — was never actually exercised: every real
//! scene built with this engine (crate stacking, kinematic platforms,
//! orbiting bodies) only ever collided simple convex primitives, exactly
//! `avian3d`'s own native strength, while this engine still lacks joints,
//! real CCD, and sleeping that `avian3d` ships out of the box. **`avian3d`
//! (see `crate::physics_avian`) is now this project's primary physics
//! engine** for anything needing correct, reliable, gameplay-relevant rigid-
//! body simulation. This module's own GPU compute path (`gpu/`) is being
//! re-scoped to visual-effects-only use (large counts of simple,
//! individually-inconsequential bodies — debris/particle-like clutter —
//! where GPU throughput is the actual goal, not simulation fidelity),
//! mirroring how GPU PhysX was historically used alongside a CPU physics
//! engine in shipped games.
//!
//! This module is kept, not deleted, as a working reference/comparison
//! baseline with its own real regression-test coverage — none of its own
//! examples (`physics_orbit.rs`, `physics_stability.rs`,
//! `physics_playground.rs`) have been migrated to `avian3d`, and none of
//! `crate::physics_avian`'s own work depends on this module continuing to
//! exist. See the plan document that recorded this decision (search git/
//! session history for "adopt avian3d as the primary physics engine") for
//! the full build-vs-buy comparison and the explicit disposal decision —
//! actual deletion of this module is a separate, later, deliberate
//! cleanup pass, not bundled into the demotion itself.
//!
//! Follows `src/hybrid`'s own mandatory methodology: every non-trivial
//! formula (contact generation, XPBD constraint math, inertia tensors) gets
//! a CPU-testable reference with `cargo test` cases before any WGSL port —
//! see `src/hybrid/mod.rs`'s doc comment for why that discipline exists.
//!
//! `physics` is scene-authoritative data + systems, not renderer-owned,
//! mirroring the existing `sdf` (data) / `hybrid` (consumer) split: this
//! module writes `Transform` on `Shape` entities, and `src/hybrid` reads it
//! exactly as it already does for any other animated shape.

pub mod broadphase;
pub mod collision_static;
pub mod components;
pub mod contacts;
pub mod gpu;
pub mod inertia;
pub mod integrate;
pub mod sample_points;
pub mod solve_rigid;
pub mod solve_static;
pub mod solve_world;
