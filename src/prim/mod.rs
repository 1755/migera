//! Shape geometry utilities shared across the hybrid renderer: axis-aligned
//! bounds computation for BVH construction.
//!
//! Shapes are carried by [`GpuPrimitive`](crate::sdf::primitives::GpuPrimitive)
//! (kind + flat params) so parameters have ONE source of truth across the
//! legacy GPU record path and this module.
//!
//! An earlier exact-analytic-ray-intersection tier (`distance`/`intersect`
//! dual-form dispatch, `Span`/`Spans` interval algebra, `error_bound`) lived
//! here; it was removed after profiling showed it cost 5-6x the GPU dispatch
//! time of SDF marching for lit/shadowed/reflective objects. See
//! `docs/knowledge/analytic-intersections/INDEX.md` for the finding — the
//! research documents there remain a valid reference on analytic
//! intersection theory even though migera no longer uses it in production.

use bevy::math::{Quat, Vec3};

pub use crate::sdf::primitives::{GpuPrimitive, GpuShapeKind};

/// Axis-aligned bounds in a shape's local space.
#[derive(Clone, Copy, Debug)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    pub fn from_center_half(center: Vec3, half: Vec3) -> Self {
        Self {
            min: center - half,
            max: center + half,
        }
    }

    /// Huge finite bound standing in for unbounded extents (infinite repeats,
    /// planes). Finite so tests stay well-defined; far beyond any scene scale.
    pub const UNBOUNDED_HALF_EXTENT: f32 = 1e6;

    pub fn united(&self, other: &Aabb) -> Aabb {
        Aabb {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// Conservative intersection (never smaller than the true one).
    pub fn intersected(&self, other: &Aabb) -> Aabb {
        Aabb {
            min: self.min.max(other.min),
            max: self.max.min(other.max),
        }
    }

    pub fn expanded(&self, margin: f32) -> Aabb {
        Aabb {
            min: self.min - Vec3::splat(margin),
            max: self.max + Vec3::splat(margin),
        }
    }

    /// Conservative world-space bound of this box under a translate-then-
    /// rotate placement: rotate all 8 corners, take extremes.
    pub fn transformed(&self, translation: Vec3, rotation: Quat) -> Aabb {
        let mut min = Vec3::splat(f32::INFINITY);
        let mut max = Vec3::splat(f32::NEG_INFINITY);
        for sx in [0.0f32, 1.0] {
            for sy in [0.0, 1.0] {
                for sz in [0.0, 1.0] {
                    let corner = Vec3::new(
                        self.min.x + (self.max.x - self.min.x) * sx,
                        self.min.y + (self.max.y - self.min.y) * sy,
                        self.min.z + (self.max.z - self.min.z) * sz,
                    );
                    let rotated = rotation * corner + translation;
                    min = min.min(rotated);
                    max = max.max(rotated);
                }
            }
        }
        Aabb { min, max }
    }

    pub fn contains(&self, p: Vec3) -> bool {
        p.x >= self.min.x
            && p.y >= self.min.y
            && p.z >= self.min.z
            && p.x <= self.max.x
            && p.y <= self.max.y
            && p.z <= self.max.z
    }
}

// ---------------------------------------------------------------------------
// Local-space AABBs
// ---------------------------------------------------------------------------

fn vec3_params(prim: &GpuPrimitive, base: usize) -> Vec3 {
    Vec3::new(
        prim.params[base],
        prim.params[base + 1],
        prim.params[base + 2],
    )
}

pub fn local_aabb(prim: &GpuPrimitive) -> Aabb {
    match prim.kind {
        GpuShapeKind::Sphere => Aabb::from_center_half(Vec3::ZERO, Vec3::splat(prim.params[0])),
        GpuShapeKind::RoundedBox => Aabb::from_center_half(Vec3::ZERO, vec3_params(prim, 0)),
        GpuShapeKind::RoundedCylinder => Aabb::from_center_half(
            Vec3::ZERO,
            Vec3::new(prim.params[0], prim.params[1], prim.params[0]),
        ),
        GpuShapeKind::Capsule => {
            let (a, b) = (vec3_params(prim, 0), vec3_params(prim, 3));
            let r = prim.params[6];
            Aabb {
                min: a.min(b) - Vec3::splat(r),
                max: a.max(b) + Vec3::splat(r),
            }
        }
        GpuShapeKind::RoundedCone => {
            let (a, b) = (vec3_params(prim, 0), vec3_params(prim, 3));
            let rr = Vec3::splat(prim.params[6].max(prim.params[7]));
            Aabb {
                min: a.min(b) - rr,
                max: a.max(b) + rr,
            }
        }
        GpuShapeKind::Ellipsoid => Aabb::from_center_half(Vec3::ZERO, vec3_params(prim, 0)),
        GpuShapeKind::BoxFrame => Aabb::from_center_half(Vec3::ZERO, vec3_params(prim, 0)),
        GpuShapeKind::HexPrism => {
            // Conservative: |x| ≤ r, |z| ≤ 2r/√3, |y| ≤ hh.
            let r = prim.params[0];
            Aabb::from_center_half(
                Vec3::ZERO,
                Vec3::new(r, prim.params[1], 2.0 * r / 3f32.sqrt()),
            )
        }
    }
}
