//! Physics ECS components: rigid-body velocity state, mass/inertia storage,
//! and `PhysicsShape` — the collider-shape subset of `sdf::components::Shape`
//! that physics is allowed to touch.
//!
//! `PhysicsShape` mirrors `Shape` exactly except it has **no `RoundedCone`
//! variant at all** — not a runtime-rejected case, an unrepresentable one.
//! `sdf::primitives::RoundedCone::distance` has a real, tracked, unfixed
//! correctness bug (every point tested, including the shape's own centerline
//! and endpoints, reports exterior — see `hybrid::cpu_ref::local_distance`'s
//! doc comment), and `RoundedCone` has no GPU shape-kind tag either (it
//! silently degrades to a zero-radius sphere in `hybrid::extract`). Physics
//! must not build mass/inertia/collision math on top of a shape whose own
//! distance function is known-wrong, so the exclusion is enforced at the
//! type level: code that pattern-matches `PhysicsShape` exhaustively simply
//! cannot mishandle `RoundedCone`, because there is no such arm to write.

use bevy::prelude::*;

use crate::sdf::components::Shape;

/// A physics-enabled rigid body's linear/angular velocity, in world space.
/// Position/orientation live in the entity's own `Transform` (reused, not
/// duplicated — see this module's doc comment and `mod.rs`'s note that
/// physics writes `Transform` directly, exactly like any other animated
/// `Shape` entity).
///
/// For a `BodyKind::Kinematic` body, `linear_velocity`/`angular_velocity`
/// are NOT computed by the solver — the external system driving this
/// body's `Transform` (animation, script, cutscene) is responsible for
/// keeping these fields consistent with that motion every frame it moves
/// the body, so that dynamic bodies resting on/against it receive correct
/// contact-velocity response. A kinematic body's `Transform` is never
/// written by `solve_world`/the GPU physics path regardless of contacts.
/// The solver deliberately never derives a kinematic body's velocity from
/// consecutive-frame `Transform` deltas — `solve_world`'s own substep loop
/// samples body state once per FRAME, not once per SUBSTEP, so a derived
/// velocity would be constant garbage across all 8 substeps regardless of
/// whether the body's true motion just started, stopped, or reversed that
/// frame, and a delta can never distinguish "moving at this instantaneous
/// rate" from "teleported this frame" (a legitimate kinematic use case,
/// e.g. a door that snaps open) the way an explicit velocity write can.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq)]
pub struct RigidBody {
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

/// A body's classification for the solver: `Dynamic` bodies are moved by
/// forces/contacts and have their `Transform` written by the solver every
/// frame; `Kinematic` bodies carry a real `RigidBody` velocity and impart
/// it to dynamic bodies on contact, but are immovable by any correction
/// (`Inertia::STATIC`-shaped mass) and have their `Transform` driven
/// externally, never by the solver; `Static` bodies never move and never
/// carry `RigidBody` at all (unchanged from before this enum existed —
/// present here only for query-symmetry, not a new requirement, since a
/// static collider is still identified the same way it always was: a
/// `PhysicsShape` entity with no `RigidBody`).
///
/// Defaults to `Dynamic` and is fetched everywhere via
/// `Option<&BodyKind>` + `.copied().unwrap_or_default()` rather than
/// requiring every existing dynamic-body spawn site (tests, examples) to
/// add it explicitly — this keeps every pre-existing spawn bundle
/// unchanged and behaviorally identical.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BodyKind {
    #[default]
    Dynamic,
    Kinematic,
    Static,
}

/// Inverse mass and inverse inertia tensor (body-local, diagonal — every
/// current primitive's closed-form inertia is diagonal in its own local
/// frame, see `physics::inertia`), stored inverted per XPBD convention
/// (Müller et al., "Detailed Rigid Body Simulation with Extended Position
/// Based Dynamics", SCA/CGF 2020): every constraint correction scales by
/// inverse mass/inertia, and `0.0` is the natural, division-free way to
/// express "infinite mass" for static/kinematic bodies — no special-casing
/// a `Static` marker component through the solver's math.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct Inertia {
    pub inverse_mass: f32,
    /// Diagonal of the body-local inverse inertia tensor.
    pub inverse_tensor_diag: Vec3,
}

impl Inertia {
    /// Infinite mass/inertia — the static/kinematic case: zero inverse mass
    /// and inverse inertia, so any constraint correction scaled by these
    /// values is naturally zero, with no branch needed at any call site.
    pub const STATIC: Self = Self { inverse_mass: 0.0, inverse_tensor_diag: Vec3::ZERO };
}

/// The collider-shape subset of `sdf::components::Shape` that physics is
/// allowed to touch — see this module's doc comment for why `RoundedCone`
/// has no variant here at all.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub enum PhysicsShape {
    Sphere { radius: f32 },
    RoundedBox { half_extents: Vec3, corner_radius: f32 },
    RoundedCylinder { radius: f32, half_height: f32, edge_radius: f32 },
    Capsule { a: Vec3, b: Vec3, radius: f32 },
    Ellipsoid { radii: Vec3 },
    BoxFrame { half_extents: Vec3, wall_thickness: f32 },
    HexPrism { radius: f32, half_height: f32 },
}

/// `Shape::RoundedCone` has no `PhysicsShape` counterpart — see this
/// module's doc comment. Every other variant round-trips exactly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RoundedConeUnsupported;

impl TryFrom<Shape> for PhysicsShape {
    type Error = RoundedConeUnsupported;

    fn try_from(shape: Shape) -> Result<Self, Self::Error> {
        match shape {
            Shape::Sphere { radius } => Ok(Self::Sphere { radius }),
            Shape::RoundedBox { half_extents, corner_radius } => {
                Ok(Self::RoundedBox { half_extents, corner_radius })
            }
            Shape::RoundedCylinder { radius, half_height, edge_radius } => {
                Ok(Self::RoundedCylinder { radius, half_height, edge_radius })
            }
            Shape::Capsule { a, b, radius } => Ok(Self::Capsule { a, b, radius }),
            Shape::RoundedCone { .. } => Err(RoundedConeUnsupported),
            Shape::Ellipsoid { radii } => Ok(Self::Ellipsoid { radii }),
            Shape::BoxFrame { half_extents, wall_thickness } => {
                Ok(Self::BoxFrame { half_extents, wall_thickness })
            }
            Shape::HexPrism { radius, half_height } => Ok(Self::HexPrism { radius, half_height }),
        }
    }
}

impl From<PhysicsShape> for Shape {
    fn from(shape: PhysicsShape) -> Self {
        match shape {
            PhysicsShape::Sphere { radius } => Self::Sphere { radius },
            PhysicsShape::RoundedBox { half_extents, corner_radius } => {
                Self::RoundedBox { half_extents, corner_radius }
            }
            PhysicsShape::RoundedCylinder { radius, half_height, edge_radius } => {
                Self::RoundedCylinder { radius, half_height, edge_radius }
            }
            PhysicsShape::Capsule { a, b, radius } => Self::Capsule { a, b, radius },
            PhysicsShape::Ellipsoid { radii } => Self::Ellipsoid { radii },
            PhysicsShape::BoxFrame { half_extents, wall_thickness } => {
                Self::BoxFrame { half_extents, wall_thickness }
            }
            PhysicsShape::HexPrism { radius, half_height } => Self::HexPrism { radius, half_height },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rounded_cone_is_rejected() {
        let shape = Shape::RoundedCone { a: Vec3::ZERO, b: Vec3::Y, r0: 1.0, r1: 0.5 };
        assert_eq!(PhysicsShape::try_from(shape), Err(RoundedConeUnsupported));
    }

    #[test]
    fn every_other_shape_round_trips_through_physics_shape() {
        let shapes = [
            Shape::Sphere { radius: 1.0 },
            Shape::RoundedBox { half_extents: Vec3::new(1.0, 2.0, 3.0), corner_radius: 0.1 },
            Shape::RoundedCylinder { radius: 1.0, half_height: 2.0, edge_radius: 0.2 },
            Shape::Capsule { a: Vec3::ZERO, b: Vec3::Y, radius: 0.5 },
            Shape::Ellipsoid { radii: Vec3::new(1.0, 2.0, 3.0) },
            Shape::BoxFrame { half_extents: Vec3::new(1.0, 1.0, 1.0), wall_thickness: 0.1 },
            Shape::HexPrism { radius: 1.0, half_height: 0.5 },
        ];
        for shape in shapes {
            let physics_shape = PhysicsShape::try_from(shape).expect("only RoundedCone should be rejected");
            let round_tripped: Shape = physics_shape.into();
            // `Shape` has no `PartialEq` (see its own definition), so compare
            // via `Debug` formatting — sufficient to catch a field-mapping
            // mistake in either `TryFrom`/`From` impl without adding a
            // manual field-by-field match arm per variant here.
            assert_eq!(format!("{shape:?}"), format!("{round_tripped:?}"));
        }
    }

    #[test]
    fn static_inertia_has_zero_inverse_mass_and_tensor() {
        assert_eq!(Inertia::STATIC.inverse_mass, 0.0);
        assert_eq!(Inertia::STATIC.inverse_tensor_diag, Vec3::ZERO);
    }

    #[test]
    fn body_kind_defaults_to_dynamic() {
        assert_eq!(BodyKind::default(), BodyKind::Dynamic);
    }
}
