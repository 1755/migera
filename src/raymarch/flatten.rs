//! Flattens the CPU `Node` CSG tree (`sdf::scene::Node`) into a flat, GPU-uploadable
//! list of tagged primitive records — the bridge between the scene-authoring/assembly
//! layer (`sdf::assembly`, `sdf::world`) and the raymarcher's WGSL `map()` function,
//! which walks this same flat list every march step
//! (`assets/shaders/raymarch.wgsl`'s `eval_stack`), evaluating it live every pixel,
//! every frame — no baking step.
//!
//! `Node` is a binary tree (`SmoothUnion(a, b, k)` etc.); WGSL has no recursion and no
//! tree pointers. The encoding here is a **post-order (RPN) list**: `flatten_node`
//! walks `Node` exactly like `Node::distance` recurses (`scene.rs:143-152`), except it
//! *pushes* a record instead of *returning* a computed `f32` — first both children's
//! records, then one `Combine` record naming the operator. The shader evaluates this
//! with a small fixed-size value stack (see `raymarch.wgsl`'s `eval_stack`), which is
//! the standard non-recursive way to evaluate a post-order expression list. No operand
//! indices/pointers are needed in the records themselves — that's the entire point of
//! choosing RPN over a node-index encoding.

use bevy::math::{Quat, Vec3};
use bevy::render::render_resource::ShaderType;

use crate::sdf::primitives::GpuShapeKind;
use crate::sdf::scene::Node;
use crate::sdf::world::TileClusterScene;

/// `TAG_LEAF_*`/`TAG_OP_*` — mirrors `assets/shaders/raymarch.wgsl`'s identically-named
/// `const`s exactly; keep both sides in sync by hand (WGSL can't `include!` a Rust
/// const). Leaf tags are `< LEAF_TAG_COUNT`; anything else is an operator.
pub const TAG_LEAF_SPHERE: u32 = 0;
pub const TAG_LEAF_ROUNDED_BOX: u32 = 1;
pub const TAG_LEAF_ROUNDED_CYLINDER: u32 = 2;
pub const TAG_LEAF_CAPSULE: u32 = 3;
pub const TAG_LEAF_ROUNDED_CONE: u32 = 4;
pub const TAG_LEAF_ELLIPSOID: u32 = 5;
pub const TAG_LEAF_BOX_FRAME: u32 = 6;
pub const TAG_LEAF_HEX_PRISM: u32 = 7;
const LEAF_TAG_COUNT: u32 = 8;
pub const TAG_OP_UNION: u32 = 8;
pub const TAG_OP_SMOOTH_UNION: u32 = 9;
pub const TAG_OP_SUBTRACT: u32 = 10;
pub const TAG_OP_SMOOTH_SUBTRACT: u32 = 11;
/// Stage-1 op-set completion — keep in sync with `raymarch.wgsl`'s identical consts.
pub const TAG_OP_INTERSECT: u32 = 12;
pub const TAG_OP_SMOOTH_INTERSECT: u32 = 13;

fn leaf_tag(kind: GpuShapeKind) -> u32 {
    match kind {
        GpuShapeKind::Sphere => TAG_LEAF_SPHERE,
        GpuShapeKind::RoundedBox => TAG_LEAF_ROUNDED_BOX,
        GpuShapeKind::RoundedCylinder => TAG_LEAF_ROUNDED_CYLINDER,
        GpuShapeKind::Capsule => TAG_LEAF_CAPSULE,
        GpuShapeKind::RoundedCone => TAG_LEAF_ROUNDED_CONE,
        GpuShapeKind::Ellipsoid => TAG_LEAF_ELLIPSOID,
        GpuShapeKind::BoxFrame => TAG_LEAF_BOX_FRAME,
        GpuShapeKind::HexPrism => TAG_LEAF_HEX_PRISM,
    }
}

/// One flattened primitive-tree entry — either a leaf shape or a combine operator —
/// `#[derive(ShaderType)]` so `encase` (not hand-placed padding) computes the exact
/// WGSL-matching storage-buffer layout. Every leaf tag reuses the same
/// `param_a..param_h` slots for its own scalar
/// params (WGSL has no tagged unions — this is the standard "reinterpret fields by tag"
/// GPU encoding, mirroring `sdf::components::Shape`'s own variants):
/// - `TAG_LEAF_SPHERE`: `param_a` = radius
/// - `TAG_LEAF_ROUNDED_BOX`: `param_a/b/c` = half_extents.x/y/z, `param_d` = corner_radius
/// - `TAG_LEAF_ROUNDED_CYLINDER`: `param_a` = radius, `param_b` = half_height, `param_c` = edge_radius
/// - `TAG_LEAF_CAPSULE`: `param_a/b/c` = endpoint_a.x/y/z, `param_d/e/f` = endpoint_b.x/y/z, `param_g` = radius
/// - `TAG_LEAF_ROUNDED_CONE`: `param_a/b/c` = endpoint_a.x/y/z, `param_d/e/f` = endpoint_b.x/y/z, `param_g` = r0, `param_h` = r1
/// - `TAG_LEAF_ELLIPSOID`: `param_a/b/c` = radii.x/y/z
/// - `TAG_LEAF_BOX_FRAME`: `param_a/b/c` = half_extents.x/y/z, `param_d` = wall_thickness
/// - `TAG_LEAF_HEX_PRISM`: `param_a` = radius, `param_b` = half_height
///
/// For `TAG_OP_*` records, `param_a` is the blend radius `k` (unused/`0.0` for the hard
/// `TAG_OP_UNION`/`TAG_OP_SUBTRACT` variants) and every other field is unused padding.
///
/// `translation`/`rotation`/`rotation_is_identity` mirror `sdf::scene::Isometry`
/// exactly (only meaningful on leaf records) — `rotation_is_identity` mirrors
/// `Isometry::inverse_transform_point`'s own fast path (`scene.rs:45-52`, a measured
/// ~2x win at CPU-bake scale) so the shader can skip the quaternion multiply per leaf
/// per march step the same way the CPU baker skips it per leaf per bake sample.
///
/// `anim_group` is `0` for ordinary static leaves, or `group_id + 1` for a leaf that's
/// part of an `AnimGroup`'s rest-pose subtree — see `raymarch::extract`'s doc comment
/// for how the shader uses this to look up the group's live rotation.
/// `Vec3`/`Vec4` are deliberately NOT used for `translation`/`rotation` here even
/// though that's the more natural Rust type: `encase`'s `ShaderType` derive inserts
/// WGSL-alignment padding around a `vec3<f32>` field (16-byte aligned in WGSL despite
/// being a 12-byte type), which diverges from `glam::Vec3`'s own tightly-packed
/// `repr(C)` Rust layout — deriving `bytemuck::Pod` on a struct where those two derived
/// layouts disagree panics at compile time ("derive(Pod) was applied to a type with
/// padding"). Plain scalar fields keep the Rust layout unambiguous and gap-free, same
/// convention `bevy_pbr`'s own `RenderClusteredDecal` GPU record type uses. Material
/// fields below follow the same scalars-only convention for the same reason.
///
/// `pattern_id` is `0` for a plain `Material` leaf (shaded directly from
/// `base_color_*`/`metallic`/`roughness`) or a 1-based index into the frame's active-
/// pattern list (see `raymarch::pipeline::PatternRegistry`) for a `ProceduralPattern`
/// leaf, in which case the shader instead calls that pattern's dispatcher with
/// `base_color_*`/`metallic`/`roughness` as "material A", `material_b_*`/`metallic_b`/
/// `roughness_b` as "material B", and `pattern_params` as its generic tunable — see
/// `sdf::components::ProceduralPattern`'s doc comment for the full contract. Resolved
/// from each leaf's source entity by `raymarch::flatten::flatten_node` (which only
/// knows the pattern's *import path string*, not yet its registry index — GPU records
/// must stay a fixed-layout `Pod` struct, which can't hold a `String`) into a real
/// index by `raymarch::pipeline::prepare_raymarch_buffers` once the active-pattern
/// list for this frame's pipeline specialization is known — see `FlattenedScene::
/// pattern_import_paths` for the parallel per-record list that resolution reads from.
#[derive(Clone, Copy, Debug, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct PrimitiveRecordCpu {
    pub tag: u32,
    pub anim_group: u32,
    pub param_a: f32,
    pub param_b: f32,
    pub param_c: f32,
    pub param_d: f32,
    pub param_e: f32,
    pub param_f: f32,
    pub param_g: f32,
    pub param_h: f32,
    pub translation_x: f32,
    pub translation_y: f32,
    pub translation_z: f32,
    pub rotation_is_identity: u32,
    pub rotation_x: f32,
    pub rotation_y: f32,
    pub rotation_z: f32,
    pub rotation_w: f32,
    pub base_color_r: f32,
    pub base_color_g: f32,
    pub base_color_b: f32,
    pub metallic: f32,
    pub roughness: f32,
    pub pattern_id: u32,
    pub material_b_r: f32,
    pub material_b_g: f32,
    pub material_b_b: f32,
    pub metallic_b: f32,
    pub roughness_b: f32,
    pub pattern_params_x: f32,
    pub pattern_params_y: f32,
    pub pattern_params_z: f32,
    pub pattern_params_w: f32,
    pub center_x: f32,
    pub center_y: f32,
    pub center_z: f32,
    pub bounding_radius: f32,
}

/// Fallback material for a leaf with no `LeafMaterial` at all (see `Node::Leaf::
/// material`'s doc comment — this project's own hand-written `sdf::world` helper
/// functions predate per-primitive material authoring) — a neutral, mid-roughness
/// dielectric, matching the flat `SURFACE_COLOR` every primitive in this scene shaded
/// with before per-primitive materials existed.
const DEFAULT_MATERIAL: (Vec3, f32, f32) = (Vec3::new(0.72, 0.70, 0.66), 0.0, 0.5);

impl PrimitiveRecordCpu {
    pub(crate) fn op(tag: u32, blend_radius: f32) -> Self {
        Self {
            tag,
            anim_group: 0,
            param_a: blend_radius,
            param_b: 0.0,
            param_c: 0.0,
            param_d: 0.0,
            param_e: 0.0,
            param_f: 0.0,
            param_g: 0.0,
            param_h: 0.0,
            translation_x: 0.0,
            translation_y: 0.0,
            translation_z: 0.0,
            rotation_is_identity: 1,
            rotation_x: 0.0,
            rotation_y: 0.0,
            rotation_z: 0.0,
            rotation_w: 1.0,
            base_color_r: 0.0,
            base_color_g: 0.0,
            base_color_b: 0.0,
            metallic: 0.0,
            roughness: 0.0,
            pattern_id: 0,
            material_b_r: 0.0,
            material_b_g: 0.0,
            material_b_b: 0.0,
            metallic_b: 0.0,
            roughness_b: 0.0,
            pattern_params_x: 0.0,
            pattern_params_y: 0.0,
            pattern_params_z: 0.0,
            pattern_params_w: 0.0,
            center_x: 0.0,
            center_y: 0.0,
            center_z: 0.0,
            bounding_radius: 0.0,
        }
    }

    /// `pattern_id` is always written as `0` here — `flatten_node` (the only caller)
    /// resolves the real 1-based pattern registry index afterward, once the record is
    /// in its final position in `out` (see `raymarch::pipeline::prepare_raymarch_buffers`,
    /// which owns the render-world-side registry `flatten_node` itself has no access
    /// to) — this constructor only fills in what's knowable from the leaf alone.
    pub(crate) fn leaf(
        shape: &dyn crate::sdf::primitives::Sdf,
        translation: Vec3,
        rotation: Quat,
        material: (Vec3, f32, f32),
        material_b: (Vec3, f32, f32),
        pattern_params: bevy::math::Vec4,
    ) -> Self {
        let record = shape.gpu_record();
        let p = record.params;
        let (center, bounding_radius) = match record.kind {
            GpuShapeKind::Sphere => (Vec3::ZERO, p[0]),
            GpuShapeKind::RoundedBox => (Vec3::ZERO, Vec3::new(p[0], p[1], p[2]).length() + p[3]),
            GpuShapeKind::RoundedCylinder => {
                let r = p[0];
                let h = p[1];
                let e = p[2];
                (Vec3::ZERO, (r * r + (h + e) * (h + e)).sqrt())
            }
            GpuShapeKind::Capsule => {
                let a = Vec3::new(p[0], p[1], p[2]);
                let b = Vec3::new(p[3], p[4], p[5]);
                ((a + b) * 0.5, (a - b).length() * 0.5 + p[6])
            }
            GpuShapeKind::RoundedCone => {
                let a = Vec3::new(p[0], p[1], p[2]);
                let b = Vec3::new(p[3], p[4], p[5]);
                ((a + b) * 0.5, (a - b).length() * 0.5 + p[6].max(p[7]))
            }
            GpuShapeKind::Ellipsoid => (Vec3::ZERO, p[0].max(p[1]).max(p[2])),
            GpuShapeKind::BoxFrame => (Vec3::ZERO, Vec3::new(p[0], p[1], p[2]).length() + p[3]),
            GpuShapeKind::HexPrism => (Vec3::ZERO, (p[0] * p[0] + p[1] * p[1]).sqrt()),
        };
        Self {
            tag: leaf_tag(record.kind),
            anim_group: 0,
            param_a: record.params[0],
            param_b: record.params[1],
            param_c: record.params[2],
            param_d: record.params[3],
            param_e: record.params[4],
            param_f: record.params[5],
            param_g: record.params[6],
            param_h: record.params[7],
            translation_x: translation.x,
            translation_y: translation.y,
            translation_z: translation.z,
            rotation_is_identity: (rotation == Quat::IDENTITY) as u32,
            rotation_x: rotation.x,
            rotation_y: rotation.y,
            rotation_z: rotation.z,
            rotation_w: rotation.w,
            base_color_r: material.0.x,
            base_color_g: material.0.y,
            base_color_b: material.0.z,
            metallic: material.1,
            roughness: material.2,
            pattern_id: 0,
            material_b_r: material_b.0.x,
            material_b_g: material_b.0.y,
            material_b_b: material_b.0.z,
            metallic_b: material_b.1,
            roughness_b: material_b.2,
            pattern_params_x: pattern_params.x,
            pattern_params_y: pattern_params.y,
            pattern_params_z: pattern_params.z,
            pattern_params_w: pattern_params.w,
            center_x: center.x,
            center_y: center.y,
            center_z: center.z,
            bounding_radius,
        }
    }
}

/// One `AnimGroup`'s flattened rest-pose subtree, plus which records in
/// `FlattenedScene::anim_group_records` belong to it (`[start, start + count)`) — mirrors
/// `sdf::assembly::AnimGroupScene`'s `{id, node, pivot}` shape, just with `node` already
/// flattened.
pub struct FlattenedAnimGroup {
    pub id: u32,
    pub pivot: Vec3,
    pub start: u32,
    pub count: u32,
}

/// Parallel output alongside a `Vec<PrimitiveRecordCpu>`: each record's
/// `ProceduralPattern` import path, if it has one (`None` for `Solid`-material leaves
/// and all op records) — kept as a same-length side list rather than inside
/// `PrimitiveRecordCpu` itself because GPU records must stay a fixed-layout `Pod`
/// struct with no `String` field (see that struct's doc comment). Consumed by
/// `raymarch::pipeline::prepare_raymarch_buffers`, which resolves each `Some(path)`
/// into the frame's pipeline-specialization-driven pattern registry index and writes
/// the result into the corresponding record's `pattern_id` right before upload.
#[derive(Default)]
struct FlattenOutput {
    records: Vec<PrimitiveRecordCpu>,
    pattern_import_paths: Vec<Option<String>>,
}

impl FlattenOutput {
    fn push(&mut self, record: PrimitiveRecordCpu, pattern_import_path: Option<String>) {
        self.records.push(record);
        self.pattern_import_paths.push(pattern_import_path);
    }

    fn len(&self) -> usize {
        self.records.len()
    }
}

/// The whole scene, flattened and ready to upload: one contiguous `static_records` list
/// (already `repeat_xz`-tiled conceptually — `tile_period` is applied by the shader to
/// the query point before walking these, per `Node::Repeat`'s own O(1) semantics, see
/// this module's top doc comment) plus one contiguous `anim_group_records` list covering
/// every `AnimGroup`'s own rest-pose subtree, sliced per-group by `anim_groups`.
/// `static_pattern_import_paths`/`anim_group_pattern_import_paths` are the parallel
/// per-record pattern paths for each list (see `FlattenOutput`'s doc comment) — same
/// length and index correspondence as their record `Vec` counterpart.
pub struct FlattenedScene {
    pub tile_period: f32,
    pub static_records: Vec<PrimitiveRecordCpu>,
    pub static_pattern_import_paths: Vec<Option<String>>,
    pub anim_groups: Vec<FlattenedAnimGroup>,
    pub anim_group_records: Vec<PrimitiveRecordCpu>,
    pub anim_group_pattern_import_paths: Vec<Option<String>>,
}

/// Resolves a leaf's `LeafMaterial` (if any) into the `(material_a, material_b,
/// pattern_params, pattern_import_path)` tuple `PrimitiveRecordCpu::leaf` and
/// `FlattenOutput::push` need — `None` (no `LeafMaterial` at all, see `Node::Leaf::
/// material`'s doc comment) falls back to `DEFAULT_MATERIAL` for both material slots.
fn resolve_leaf_material(
    material: &Option<crate::sdf::scene::LeafMaterial>,
) -> (
    (Vec3, f32, f32),
    (Vec3, f32, f32),
    bevy::math::Vec4,
    Option<String>,
) {
    use crate::sdf::scene::LeafMaterial;
    match material {
        None => (
            DEFAULT_MATERIAL,
            DEFAULT_MATERIAL,
            bevy::math::Vec4::ZERO,
            None,
        ),
        Some(LeafMaterial::Solid {
            base_color,
            metallic,
            roughness,
        }) => (
            (*base_color, *metallic, *roughness),
            DEFAULT_MATERIAL,
            bevy::math::Vec4::ZERO,
            None,
        ),
        Some(LeafMaterial::Pattern {
            import_path,
            material_a,
            material_b,
            params,
            ..
        }) => (*material_a, *material_b, *params, Some(import_path.clone())),
    }
}

/// Recursively flattens `node` into post-order (RPN) records appended to `out` — a
/// direct structural mirror of `Node::distance`'s own recursion (`scene.rs:143-152`):
/// recurse into children first (so their records precede the combining op, matching
/// post-order/RPN evaluation order), then push one record for this node itself.
///
/// `Node::Repeat` is deliberately not handled here — see `flatten_scene`, which expects
/// it only at the very root and strips it before calling this function, matching
/// `assemble_infinite_scene`'s guarantee that `Repeat` only ever wraps the whole static
/// scene once (`world.rs:287`), never nested deeper.
fn flatten_node(node: &Node, out: &mut FlattenOutput) {
    match node {
        Node::Leaf {
            shape,
            transform,
            material,
        } => {
            let (material_a, material_b, pattern_params, pattern_import_path) =
                resolve_leaf_material(material);
            let record = PrimitiveRecordCpu::leaf(
                shape.as_ref(),
                transform.translation,
                transform.rotation,
                material_a,
                material_b,
                pattern_params,
            );
            out.push(record, pattern_import_path);
        }
        Node::Union(a, b) => {
            flatten_node(a, out);
            flatten_node(b, out);
            out.push(PrimitiveRecordCpu::op(TAG_OP_UNION, 0.0), None);
        }
        Node::SmoothUnion(a, b, k) => {
            flatten_node(a, out);
            flatten_node(b, out);
            out.push(PrimitiveRecordCpu::op(TAG_OP_SMOOTH_UNION, *k), None);
        }
        Node::Intersect(a, b) => {
            flatten_node(a, out);
            flatten_node(b, out);
            out.push(PrimitiveRecordCpu::op(TAG_OP_INTERSECT, 0.0), None);
        }
        Node::SmoothIntersect(a, b, k) => {
            flatten_node(a, out);
            flatten_node(b, out);
            out.push(PrimitiveRecordCpu::op(TAG_OP_SMOOTH_INTERSECT, *k), None);
        }
        Node::Subtract(a, b) => {
            flatten_node(a, out);
            flatten_node(b, out);
            out.push(PrimitiveRecordCpu::op(TAG_OP_SUBTRACT, 0.0), None);
        }
        Node::SmoothSubtract(a, b, k) => {
            flatten_node(a, out);
            flatten_node(b, out);
            out.push(PrimitiveRecordCpu::op(TAG_OP_SMOOTH_SUBTRACT, *k), None);
        }
        Node::Repeat(_inner, period) => {
            // `assemble_infinite_scene` only ever wraps the whole static scene in one
            // top-level `Repeat` (world.rs:287) — a `Repeat` showing up *nested* inside
            // the tree would mean some other part of the tree got tiled independently,
            // which nothing in this codebase does today and which `flatten_scene`'s
            // "unwrap Repeat at the root only" contract doesn't support. Panicking here
            // (rather than silently ignoring the period, which would produce a subtly
            // wrong un-tiled scene) surfaces that assumption breaking immediately if
            // the scene-authoring side ever changes.
            panic!(
                "flatten_node found a nested Node::Repeat (period {period}) — only a \
                 top-level Repeat, stripped by flatten_scene, is supported"
            );
        }
    }
}

/// Marks every leaf record pushed while `f` runs as belonging to `anim_group` (`id +
/// 1`, `0` reserved for "no group" — see `PrimitiveRecordCpu::anim_group`'s doc
/// comment) — used by `flatten_scene` when flattening each `AnimGroupScene`'s subtree.
fn flatten_node_tagged(node: &Node, group_id: u32, out: &mut FlattenOutput) {
    let start = out.len();
    flatten_node(node, out);
    for record in &mut out.records[start..] {
        if record.tag < LEAF_TAG_COUNT {
            record.anim_group = group_id + 1;
        }
    }
}

/// Flattens a whole `TileClusterScene` (static geometry + per-`AnimGroup` rest-pose
/// subtrees) into GPU-uploadable form. Requires `scene.static_scene` to be a top-level
/// `Node::Repeat` (guaranteed by `assemble_infinite_scene`, `world.rs:287`) — panics
/// otherwise, since that invariant is already supposed to hold by construction and a
/// silent fallback would hide a real bug in the scene-assembly side instead.
pub fn flatten_scene(scene: &TileClusterScene) -> FlattenedScene {
    let Node::Repeat(inner, tile_period) = &scene.static_scene else {
        panic!(
            "TileClusterScene::static_scene must be a top-level Node::Repeat (see assemble_infinite_scene)"
        );
    };

    let mut static_out = FlattenOutput::default();
    flatten_node(inner, &mut static_out);

    let mut anim_group_out = FlattenOutput::default();
    let mut anim_groups = Vec::with_capacity(scene.anim_groups.len());
    for group in &scene.anim_groups {
        let start = anim_group_out.len() as u32;
        flatten_node_tagged(&group.node, group.id, &mut anim_group_out);
        let count = anim_group_out.len() as u32 - start;
        anim_groups.push(FlattenedAnimGroup {
            id: group.id,
            pivot: group.pivot,
            start,
            count,
        });
    }

    FlattenedScene {
        tile_period: *tile_period,
        static_records: static_out.records,
        static_pattern_import_paths: static_out.pattern_import_paths,
        anim_groups,
        anim_group_records: anim_group_out.records,
        anim_group_pattern_import_paths: anim_group_out.pattern_import_paths,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::assembly::test_app_with;
    use crate::sdf::components::{AnimGroup, Blend, BlendMode, MaterialLegacy, ProceduralPattern, Shape};
    use crate::sdf::scene::Node;
    use crate::sdf::world::{TILE_PERIOD, assemble_infinite_scene, spawn_tile_cluster};
    use bevy::ecs::system::RunSystemOnce;
    use bevy::prelude::*;

    /// Plain-Rust mirror of `raymarch.wgsl`'s `eval_stack` — same RPN stack-machine
    /// logic, once in Rust for testability (see this module's top doc comment) and once
    /// in WGSL for actual execution. Evaluates a `[start, start+count)` sub-range of
    /// `records` at world-space point `p`.
    fn eval_stack(records: &[PrimitiveRecordCpu], start: usize, count: usize, p: Vec3) -> f32 {
        let mut stack: Vec<f32> = Vec::with_capacity(8);
        for record in &records[start..start + count] {
            if record.tag < LEAF_TAG_COUNT {
                let translation = Vec3::new(
                    record.translation_x,
                    record.translation_y,
                    record.translation_z,
                );
                let local = if record.rotation_is_identity != 0 {
                    p - translation
                } else {
                    let q = Quat::from_xyzw(
                        record.rotation_x,
                        record.rotation_y,
                        record.rotation_z,
                        record.rotation_w,
                    );
                    q.inverse() * (p - translation)
                };
                let d = match record.tag {
                    TAG_LEAF_SPHERE => local.length() - record.param_a,
                    TAG_LEAF_ROUNDED_BOX => {
                        let he = Vec3::new(record.param_a, record.param_b, record.param_c);
                        let cr = record.param_d;
                        let q = local.abs() - he + Vec3::splat(cr);
                        q.max(Vec3::ZERO).length() + q.x.max(q.y.max(q.z)).min(0.0) - cr
                    }
                    TAG_LEAF_ROUNDED_CYLINDER => {
                        let radius = record.param_a;
                        let hh = record.param_b;
                        let er = record.param_c;
                        let dx = (local.x * local.x + local.z * local.z).sqrt() - radius + er;
                        let dy = local.y.abs() - hh + er;
                        dx.max(dy).min(0.0) + dx.max(0.0).hypot(dy.max(0.0)) - er
                    }
                    TAG_LEAF_CAPSULE => {
                        let a = Vec3::new(record.param_a, record.param_b, record.param_c);
                        let b = Vec3::new(record.param_d, record.param_e, record.param_f);
                        let radius = record.param_g;
                        let ab = b - a;
                        let ap = local - a;
                        let t = (ap.dot(ab) / ab.dot(ab)).clamp(0.0, 1.0);
                        let closest = a + ab * t;
                        (local - closest).length() - radius
                    }
                    TAG_LEAF_ROUNDED_CONE => {
                        let a = Vec3::new(record.param_a, record.param_b, record.param_c);
                        let b = Vec3::new(record.param_d, record.param_e, record.param_f);
                        let r0 = record.param_g;
                        let r1 = record.param_h;
                        let ba = b - a;
                        let pa = local - a;
                        let m0 = ba.dot(ba);
                        let m1 = ba.dot(pa);
                        let m2 = pa.dot(pa);
                        let d_val = m0 - m1;
                        let e = m1 - m2;
                        let f = m2 + m0 * m0 - 2.0 * m1;
                        let g = d_val * d_val * m0;
                        let h = m0 * (m0 - d_val);
                        let clamped = (d_val * e * m0 - f * h).max(0.0);
                        let t = m0 * (f * d_val - e * clamped) / (g + h * clamped) - m1;
                        let t_clamped = t.clamp(0.0, m0);
                        let q = (a + ba * t_clamped / m0 - local).length()
                            - r0
                            - (r1 - r0) * t_clamped / m0;
                        q.max(pa.length() - r0).max((local - b).length() - r1)
                    }
                    TAG_LEAF_ELLIPSOID => {
                        let radii = Vec3::new(record.param_a, record.param_b, record.param_c);
                        let k0 = (local / radii).length();
                        let k1 = (local / (radii * radii)).length();
                        k0 * (k0 - 1.0) / k1
                    }
                    TAG_LEAF_BOX_FRAME => {
                        let he = Vec3::new(record.param_a, record.param_b, record.param_c);
                        let wt = record.param_d;
                        let q = local.abs() - he;
                        let outer = q.max(Vec3::ZERO).length() + q.x.max(q.y.max(q.z)).min(0.0);
                        let inner_q = q + Vec3::splat(wt);
                        let inner = inner_q.max(Vec3::ZERO).length()
                            + inner_q.x.max(inner_q.y.max(inner_q.z)).min(0.0);
                        outer.max(-inner)
                    }
                    TAG_LEAF_HEX_PRISM => {
                        let radius = record.param_a;
                        let hh = record.param_b;
                        let q = local.abs();
                        let k = 0.866025404f32;
                        let hex_d =
                            q.x.max((0.5 * q.x + k * q.z).abs())
                                .max((0.5 * q.x - k * q.z).abs())
                                - radius;
                        let axial_d = q.y - hh;
                        hex_d.max(axial_d)
                    }
                    _ => unreachable!(),
                };
                stack.push(d);
            } else {
                let b = stack.pop().unwrap();
                let a = stack.pop().unwrap();
                let k = record.param_a;
                let result = match record.tag {
                    TAG_OP_UNION => a.min(b),
                    TAG_OP_SMOOTH_UNION => smin(a, b, k),
                    TAG_OP_INTERSECT => a.max(b),
                    TAG_OP_SMOOTH_INTERSECT => -smin(-a, -b, k),
                    TAG_OP_SUBTRACT => a.max(-b),
                    TAG_OP_SMOOTH_SUBTRACT => -smin(-a, b, k),
                    _ => unreachable!(),
                };
                stack.push(result);
            }
        }
        stack.pop().unwrap()
    }

    fn smin(a: f32, b: f32, k: f32) -> f32 {
        if k <= 0.0 {
            return a.min(b);
        }
        let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
        (b + (a - b) * h) - k * h * (1.0 - h)
    }

    #[test]
    fn golden_comparison_against_node_distance() {
        use crate::sdf::primitives::{Ellipsoid, Sphere};

        let node = Node::leaf(Sphere { radius: 1.0 }, Vec3::ZERO)
            .smooth_union(Node::leaf(Sphere { radius: 0.5 }, Vec3::X * 1.5), 0.1)
            .smooth_subtract(
                Node::leaf(
                    Ellipsoid {
                        radii: Vec3::new(0.3, 0.1, 0.3),
                    },
                    Vec3::Y * 0.5,
                ),
                0.05,
            )
            .union(Node::leaf(
                Sphere { radius: 0.2 },
                Vec3::new(-2.0, 0.0, 0.0),
            ))
            .subtract(Node::leaf(Sphere { radius: 0.1 }, Vec3::new(3.0, 0.0, 0.0)));

        let mut out = FlattenOutput::default();
        flatten_node(&node, &mut out);
        let records = out.records;

        let sample_points = [
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(-2.0, 0.0, 0.0),
            Vec3::new(3.0, 0.0, 0.0),
            Vec3::new(0.5, 0.3, -0.2),
            Vec3::new(5.0, 5.0, 5.0),
        ];
        for p in sample_points {
            let expected = node.distance(p);
            let actual = eval_stack(&records, 0, records.len(), p);
            assert!(
                (expected - actual).abs() < 1e-4,
                "mismatch at {p:?}: Node::distance={expected}, eval_stack={actual}"
            );
        }
    }

    #[test]
    fn rounded_cylinder_leaf_matches_node_distance() {
        use crate::sdf::primitives::RoundedCylinder;

        let node = Node::leaf(
            RoundedCylinder {
                radius: 0.6,
                half_height: 1.6,
                edge_radius: 0.1,
            },
            Vec3::new(1.0, 2.0, -1.0),
        );
        let mut out = FlattenOutput::default();
        flatten_node(&node, &mut out);
        let records = out.records;

        for p in [
            Vec3::ZERO,
            Vec3::new(1.0, 2.0, -1.0),
            Vec3::new(1.6, 2.0, -1.0),
            Vec3::new(1.0, 3.6, -1.0),
        ] {
            let expected = node.distance(p);
            let actual = eval_stack(&records, 0, records.len(), p);
            assert!(
                (expected - actual).abs() < 1e-4,
                "mismatch at {p:?}: {expected} vs {actual}"
            );
        }
    }

    #[test]
    fn full_scene_flatten_matches_known_surface_points() {
        let (mut world, root) = test_app_with(spawn_tile_cluster);
        let scene = world
            .run_system_once_with(
                move |shapes: Query<(
                    &Shape,
                    &GlobalTransform,
                    Option<&BlendMode>,
                    Option<&Blend>,
                    Option<&MaterialLegacy>,
                    Option<&ProceduralPattern>,
                )>,
                      children_of: Query<&Children>,
                      anim_groups: Query<&AnimGroup>,
                      transforms: Query<&GlobalTransform>| {
                    assemble_infinite_scene(root, &shapes, &children_of, &anim_groups, &transforms)
                },
                (),
            )
            .unwrap();

        let flattened = flatten_scene(&scene);

        // Same ground-truth surface point `world.rs`'s own
        // `infinite_scene_repeats_the_blob_at_the_next_cell` test uses.
        let p0 = Vec3::new(-3.0 + 1.107, 1.4, 0.0);
        let p1 = p0 + Vec3::new(TILE_PERIOD, 0.0, 0.0);
        for p in [p0, p1] {
            let wrapped = repeat_xz(p, flattened.tile_period);
            let d = eval_stack(
                &flattened.static_records,
                0,
                flattened.static_records.len(),
                wrapped,
            );
            assert!(
                d.abs() < 1e-3,
                "flattened blob point not on surface at {p:?}: {d}"
            );
        }

        // Pillar rest-pose surface point, same as `pillar_bakes_at_rest_pose_relative_to_its_own_pivot`:
        // the pillar carries no extra rotation, so its cylinder surface sits at
        // local (radius, 0, 0) = (0.6, 0, 0) relative to the pivot.
        let pillar = flattened
            .anim_groups
            .iter()
            .find(|g| g.id == 0)
            .expect("pillar's FlattenedAnimGroup should be present");
        assert_eq!(pillar.pivot, Vec3::new(4.2, 1.6, -1.0));
        let p = Vec3::new(0.6, 0.0, 0.0);
        let d = eval_stack(
            &flattened.anim_group_records,
            pillar.start as usize,
            pillar.count as usize,
            p,
        );
        assert!(
            d.abs() < 1e-3,
            "flattened pillar rest-pose point not on surface: {d}"
        );
    }

    #[test]
    fn anim_group_records_are_tagged_and_static_records_are_not() {
        let (mut world, root) = test_app_with(spawn_tile_cluster);
        let scene = world
            .run_system_once_with(
                move |shapes: Query<(
                    &Shape,
                    &GlobalTransform,
                    Option<&BlendMode>,
                    Option<&Blend>,
                    Option<&MaterialLegacy>,
                    Option<&ProceduralPattern>,
                )>,
                      children_of: Query<&Children>,
                      anim_groups: Query<&AnimGroup>,
                      transforms: Query<&GlobalTransform>| {
                    assemble_infinite_scene(root, &shapes, &children_of, &anim_groups, &transforms)
                },
                (),
            )
            .unwrap();
        let flattened = flatten_scene(&scene);

        assert!(
            flattened.static_records.iter().all(|r| r.anim_group == 0),
            "static records must not carry an anim_group tag"
        );
        assert_eq!(flattened.anim_groups.len(), 1, "expected just the pillar");
        for group in &flattened.anim_groups {
            let slice = &flattened.anim_group_records
                [group.start as usize..(group.start + group.count) as usize];
            for record in slice {
                if record.tag < LEAF_TAG_COUNT {
                    assert_eq!(
                        record.anim_group,
                        group.id + 1,
                        "leaf in group {} tagged {} instead",
                        group.id,
                        record.anim_group
                    );
                }
            }
        }
    }

    #[test]
    fn repeat_period_matches_world_tile_period() {
        let (mut world, root) = test_app_with(spawn_tile_cluster);
        let scene = world
            .run_system_once_with(
                move |shapes: Query<(
                    &Shape,
                    &GlobalTransform,
                    Option<&BlendMode>,
                    Option<&Blend>,
                    Option<&MaterialLegacy>,
                    Option<&ProceduralPattern>,
                )>,
                      children_of: Query<&Children>,
                      anim_groups: Query<&AnimGroup>,
                      transforms: Query<&GlobalTransform>| {
                    assemble_infinite_scene(root, &shapes, &children_of, &anim_groups, &transforms)
                },
                (),
            )
            .unwrap();
        let flattened = flatten_scene(&scene);
        assert_eq!(flattened.tile_period, TILE_PERIOD);
    }

    fn repeat_xz(p: Vec3, period: f32) -> Vec3 {
        let half = period * 0.5;
        Vec3::new(
            (p.x + half).rem_euclid(period) - half,
            p.y,
            (p.z + half).rem_euclid(period) - half,
        )
    }
}
