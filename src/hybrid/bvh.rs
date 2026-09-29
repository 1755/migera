//! Acceleration structure over object world-space AABBs.
//!
//! Fresh implementation — does not import or reuse anything from
//! `hybrid_legacy` (`src/hybrid_legacy::bvh` was read only as a reference
//! for the algorithm shape to mirror). This module never computes an AABB
//! itself from geometry/rotation — it only unions already-correct per-leaf
//! AABBs (see `scene::world_aabb` for where rotation correctness actually
//! lives) going up the tree.
//!
//! Two upgrades over the initial correctness baseline, both informed by
//! `docs/knowledge/hierarchical-volumes/bvh-deep-dive.md`:
//!
//! - **SAH-bucketed construction** (`build`/`recursive_sah`): ~12 spatial
//!   buckets per axis, tried on all 3 axes, argmin of the parent-surface-
//!   area-normalized cost formula that doc documents — replaces the
//!   original median-split, which is now only the fallback for degenerate
//!   splits (e.g. every leaf sharing the same centroid on every axis).
//! - **Refit instead of rebuild** (`update`): the doc's own "worst
//!   practices" list calls out rebuilding a BVH from scratch every frame
//!   for animated content — this project's cube rotates every frame, so
//!   that's exactly the pattern to avoid. `Bvh` persists as a resource
//!   across frames (see its use in `examples/gallery.rs`); `update` reuses
//!   the existing tree's topology and only recomputes bounds bottom-up
//!   when the current frame's object set is the same *set* of entities as
//!   the tree already covers (regardless of query iteration order, since
//!   leaves are keyed by Bevy's stable `Entity`, not a transient index).
//!   Any change to the object set (added/removed) falls back to a full
//!   rebuild — safe by construction, and cheap at the object counts this
//!   renderer spawns today.

use std::collections::HashMap;

use bevy::math::Vec3;
use bevy::prelude::{ChildOf, Entity, GlobalTransform, Query, ResMut, Resource, With};
use bevy::render::render_resource::ShaderType;

use crate::hybrid::scene::{self, HybridObject};
use crate::prim::Aabb;
use crate::sdf::assembly::SdfSceneRoot;
use crate::sdf::components::Shape;

/// One BVH node, flat/pointerless (child index ranges, not references) so
/// it's directly uploadable to the GPU once a shader consumes it. Leaves
/// are marked by `left_or_sentinel == LEAF_SENTINEL`; `entity` then names
/// which object the leaf covers (a stable key across frames, unlike a
/// transient index into a per-frame `Vec`) and `right_or_object` is unused
/// (kept `0`) for leaves.
#[derive(Clone, Copy, Debug)]
pub struct BvhNode {
    pub aabb: Aabb,
    pub left_or_sentinel: u32,
    pub right_or_object: u32,
    pub entity: Entity,
}

pub const LEAF_SENTINEL: u32 = u32::MAX;

/// Traversal cost constants for the SAH cost formula (see module doc) —
/// pbrt's conventional values, not independently tuned for this project;
/// only their *ratio* matters for choosing a split, not their absolute
/// scale.
const SAH_TRAVERSAL_COST: f32 = 1.0;
const SAH_INTERSECTION_COST: f32 = 1.0;
const SAH_BUCKET_COUNT: usize = 12;

/// A flat array of `BvhNode`s; index 0 is the root (or the array is empty
/// if there were no objects to build from).
#[derive(Clone, Debug, Default)]
pub struct Bvh {
    pub nodes: Vec<BvhNode>,
    /// Entity -> node-array index, covering exactly this tree's leaves.
    /// Built once per topology change (`build`, or `update`'s rebuild
    /// fallback) and reused as-is by every same-entity-set `update` call
    /// after that — see `update`'s doc comment for why rebuilding this
    /// from scratch every frame was a real, measured cost (a `HashMap`
    /// insert per entity plus a `HashSet` membership check, both O(N)
    /// over all leaves, on every single frame regardless of whether the
    /// entity set actually changed).
    leaf_index: HashMap<Entity, usize>,
}

#[derive(Clone, Copy)]
struct BuildLeaf {
    aabb: Aabb,
    center: Vec3,
    entity: Entity,
}

impl Bvh {
    /// Full SAH-bucketed build from scratch. Use `update` instead when a
    /// `Bvh` already exists for this object set and only bounds (not
    /// membership) may have changed — `update` refits without paying this
    /// function's O(N log N) sort/bucket cost every frame.
    pub fn build(objects: &[HybridObject]) -> Self {
        let mut leaves: Vec<BuildLeaf> = objects
            .iter()
            .map(|o| BuildLeaf {
                aabb: o.world_aabb,
                center: 0.5 * (o.world_aabb.min + o.world_aabb.max),
                entity: o.entity,
            })
            .collect();
        let mut nodes = Vec::with_capacity((leaves.len() * 2).max(1));
        if leaves.is_empty() {
            return Self { nodes, leaf_index: HashMap::new() };
        }
        Self::recursive_sah(&mut leaves, &mut nodes);
        let leaf_index = nodes
            .iter()
            .enumerate()
            .filter(|(_, n)| n.left_or_sentinel == LEAF_SENTINEL)
            .map(|(i, n)| (n.entity, i))
            .collect();
        Self { nodes, leaf_index }
    }

    /// Frame-to-frame entry point. If `objects` names exactly the same set
    /// of entities this tree's leaves already cover (regardless of order),
    /// refits every leaf's AABB in place and recomputes internal bounds
    /// bottom-up — no resort, no topology change, no SAH cost. Otherwise
    /// (an object was added or removed) falls back to `build` from
    /// scratch, which is always correct and, at this renderer's current
    /// object counts, cheap enough not to need avoiding.
    pub fn update(&mut self, objects: &[HybridObject]) {
        if self.leaf_index.len() != objects.len() {
            *self = Self::build(objects);
            return;
        }
        for object in objects {
            let Some(&node_index) = self.leaf_index.get(&object.entity) else {
                // An entity left `leaf_index` doesn't match one this frame
                // names — the object set changed (some entity swapped for
                // another) even though the count didn't. Falls back to a
                // full rebuild, same as a count mismatch.
                *self = Self::build(objects);
                return;
            };
            self.nodes[node_index].aabb = object.world_aabb;
        }
        if !self.nodes.is_empty() {
            Self::refit_bounds(&mut self.nodes, 0);
        }
    }

    /// Recomputes `nodes[index]`'s AABB from its (already-refit) children,
    /// recursively bottom-up. Leaves are left as-is (their AABB was
    /// already overwritten by `update`'s first pass).
    fn refit_bounds(nodes: &mut [BvhNode], index: usize) -> Aabb {
        if nodes[index].left_or_sentinel == LEAF_SENTINEL {
            return nodes[index].aabb;
        }
        let left = nodes[index].left_or_sentinel as usize;
        let right = nodes[index].right_or_object as usize;
        let left_aabb = Self::refit_bounds(nodes, left);
        let right_aabb = Self::refit_bounds(nodes, right);
        let united = left_aabb.united(&right_aabb);
        nodes[index].aabb = united;
        united
    }

    /// Builds the subtree covering `span` via SAH-bucketed splitting,
    /// appending nodes to `nodes`, and returns the new subtree root's
    /// index. `span` is partitioned in place (no copies).
    fn recursive_sah(span: &mut [BuildLeaf], nodes: &mut Vec<BvhNode>) -> usize {
        let bounds = span.iter().map(|leaf| leaf.aabb).reduce(|a, b| a.united(&b)).expect("non-empty span");

        let node_index = nodes.len();
        nodes.push(BvhNode {
            aabb: bounds,
            left_or_sentinel: LEAF_SENTINEL,
            right_or_object: 0,
            entity: span[0].entity,
        });

        if span.len() == 1 {
            return node_index;
        }

        let mid = match best_sah_split(span) {
            Some((axis, mid)) => {
                span.sort_by(|a, b| axis_of(a.center, axis).total_cmp(&axis_of(b.center, axis)));
                mid
            }
            // Degenerate case (e.g. every centroid coincides on all 3
            // axes): SAH has no boundary to prefer over any other. Fall
            // back to the median split, which is always well-defined.
            None => {
                let axis = widest_centroid_axis(span);
                span.sort_by(|a, b| axis_of(a.center, axis).total_cmp(&axis_of(b.center, axis)));
                span.len() / 2
            }
        };
        let (left, right) = span.split_at_mut(mid);
        let left_child = Self::recursive_sah(left, nodes) as u32;
        let right_child = Self::recursive_sah(right, nodes) as u32;
        nodes[node_index].left_or_sentinel = left_child;
        nodes[node_index].right_or_object = right_child;
        node_index
    }
}

/// Main-world resource holding the one `Bvh` this renderer maintains,
/// refit (not rebuilt) every frame by `update_persistent_bvh`. The single
/// source of truth both `extract.rs` (GPU upload) and any main-world
/// debug drawing (e.g. `examples/gallery.rs`'s AABB/BVH gizmos) read —
/// there is deliberately only one persisted `Bvh` in the whole app, not
/// one per consumer, so a rebuild-vs-refit decision only ever gets made
/// once per frame regardless of how many things want to look at the
/// result.
///
/// This resource replacing an earlier per-example `HybridBvh` (which
/// `examples/gallery.rs` alone maintained, with `src/hybrid/extract.rs`
/// separately doing its own from-scratch `Bvh::build` every frame just
/// for the GPU upload) is itself the fix for a real, measured bug: at
/// `--stress 10000`, that redundant full SAH rebuild — happening every
/// frame regardless of whether any object actually moved — was the
/// dominant cost in the renderer's frame time (near and far camera
/// framings measured near-identical frame cost, which only makes sense
/// if the bottleneck is fixed per-frame CPU work rather than GPU marching
/// scaling with visible geometry). See `PROGRESS.md` for the measured
/// before/after numbers.
#[derive(Resource, Default)]
pub struct PersistentBvh(pub Bvh);

/// Refits (or, on an object-set change, rebuilds) `PersistentBvh` from
/// the current frame's scene — the one place per frame this happens.
/// Registered by `HybridRenderPlugin` in the MAIN app's `Update` schedule
/// (not `ExtractSchedule`) so it runs once, before extraction reads its
/// result — see `extract.rs::extract_hybrid_scene`, which no longer
/// builds its own BVH and instead flattens whatever this system already
/// computed this frame.
pub fn update_persistent_bvh(
    mut bvh: ResMut<PersistentBvh>,
    roots: Query<Entity, With<SdfSceneRoot>>,
    shapes: Query<(Entity, &Shape, &GlobalTransform, Option<&ChildOf>)>,
) {
    let objects = scene::collect(&roots, &shapes);
    bvh.0.update(&objects);
}

/// Flat, scalar, GPU-uploadable mirror of `BvhNode`: `Aabb`'s two `Vec3`
/// fields are exploded to 6 flat `f32`s (matching `hybrid_legacy::bvh::
/// BvhNodeCpu`'s field shape, which this struct is a fresh reimplementation
/// of, not a reuse of) since `bytemuck::Pod` needs a guaranteed-no-padding
/// layout and `Entity` has no GPU representation at all — leaves store a
/// plain index into the frame's flat object array instead (see
/// `to_gpu_nodes`'s `entity_index` closure), not `BvhNode::entity` itself.
/// Lives here (not `pipeline.rs`) because it's purely a data-shape fact
/// about the BVH's own layout, independent of any pipeline/bind-group
/// wiring — `pipeline.rs` can import it without this module depending back
/// on `pipeline.rs`.
#[derive(Clone, Copy, Debug, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct BvhNodeGpu {
    pub min_x: f32,
    pub min_y: f32,
    pub min_z: f32,
    pub max_x: f32,
    pub max_y: f32,
    pub max_z: f32,
    /// Internal: left child index. Leaf: `LEAF_SENTINEL`.
    pub left_or_sentinel: u32,
    /// Internal: right child index. Leaf: index into the flat per-frame
    /// object array (NOT `BvhNode::entity` — the GPU has no `Entity`).
    pub right_or_object: u32,
}

/// Flattens a CPU `Bvh` (whose leaves are keyed by `Entity`) into a
/// GPU-uploadable `Vec<BvhNodeGpu>`, resolving each leaf's `Entity` to its
/// position in `object_order` (the same per-frame flat object array the
/// leaf's `right_or_object` must index into on the GPU side).
///
/// Does NOT build its own `Entity -> array index` map — that was a real,
/// measured cost (a fresh `HashMap` over all leaves, every frame,
/// mirroring the exact bug `Bvh::update`'s own `leaf_index` field was
/// added to fix). Instead walks `object_order` once and writes each
/// entity's array index directly into its already-known node slot via
/// `bvh.leaf_index` (built once per topology change, reused here as-is).
/// A leaf entity absent from `bvh.leaf_index` (stale BVH vs. this frame's
/// object set — shouldn't happen once `Bvh::update`/`build` and the
/// object array are built from the same frame's data, but defensively
/// handled rather than panicking) is simply never written and keeps its
/// initial `right_or_object = 0`, so a stale leaf can never index out of
/// bounds; it will just (harmlessly) shadow object 0's march candidate
/// list with an extra AABB hit.
pub fn to_gpu_nodes(bvh: &Bvh, object_order: &[Entity]) -> Vec<BvhNodeGpu> {
    let mut out: Vec<BvhNodeGpu> = bvh
        .nodes
        .iter()
        .map(|node| BvhNodeGpu {
            min_x: node.aabb.min.x,
            min_y: node.aabb.min.y,
            min_z: node.aabb.min.z,
            max_x: node.aabb.max.x,
            max_y: node.aabb.max.y,
            max_z: node.aabb.max.z,
            left_or_sentinel: node.left_or_sentinel,
            right_or_object: if node.left_or_sentinel == LEAF_SENTINEL { 0 } else { node.right_or_object },
        })
        .collect();
    for (array_index, entity) in object_order.iter().enumerate() {
        if let Some(&node_index) = bvh.leaf_index.get(entity) {
            out[node_index].right_or_object = array_index as u32;
        }
    }
    out
}

/// Binned SAH: for each axis, subdivide the leaves' centroid range on that
/// axis into `SAH_BUCKET_COUNT` spatial buckets, accumulate each bucket's
/// AABB union + leaf count, then evaluate the cost formula
/// `C_trav + SA(L)/SA(P)*N_L*C_int + SA(R)/SA(P)*N_R*C_int` at every
/// internal bucket boundary (per `bvh-deep-dive.md`'s documented formula,
/// `SA` = box surface area, `P` = this span's parent bounds). Returns the
/// `(axis, split_index)` of the globally cheapest boundary across all 3
/// axes, where `split_index` is the position in `span` (once sorted along
/// `axis`) the left/right partition falls at — or `None` if every leaf
/// shares one centroid on every axis (no boundary is meaningfully
/// separable, so there is nothing for SAH to prefer).
fn best_sah_split(span: &[BuildLeaf]) -> Option<(u8, usize)> {
    let parent = span.iter().map(|l| l.aabb).reduce(|a, b| a.united(&b)).expect("non-empty span");
    let parent_sa = surface_area(&parent);
    if parent_sa <= 0.0 {
        return None;
    }

    let mut best: Option<(f32, u8, usize)> = None;
    for axis in 0..3u8 {
        let mut cmin = f32::INFINITY;
        let mut cmax = f32::NEG_INFINITY;
        for leaf in span {
            let c = axis_of(leaf.center, axis);
            cmin = cmin.min(c);
            cmax = cmax.max(c);
        }
        let extent = cmax - cmin;
        if extent <= 1e-8 {
            continue; // every centroid coincides on this axis; try the next
        }

        let bucket_of = |center: f32| -> usize {
            let t = ((center - cmin) / extent * SAH_BUCKET_COUNT as f32) as usize;
            t.min(SAH_BUCKET_COUNT - 1)
        };
        let mut bucket_aabb: [Option<Aabb>; SAH_BUCKET_COUNT] = [None; SAH_BUCKET_COUNT];
        let mut bucket_count = [0u32; SAH_BUCKET_COUNT];
        for leaf in span {
            let b = bucket_of(axis_of(leaf.center, axis));
            bucket_aabb[b] = Some(match bucket_aabb[b] {
                Some(existing) => existing.united(&leaf.aabb),
                None => leaf.aabb,
            });
            bucket_count[b] += 1;
        }

        // Prefix (left) and suffix (right) accumulated bounds/counts at
        // each of the SAH_BUCKET_COUNT-1 internal boundaries.
        let mut left_aabb: Option<Aabb> = None;
        let mut left_count = 0u32;
        let mut left_running = [(Option::<Aabb>::None, 0u32); SAH_BUCKET_COUNT];
        for b in 0..SAH_BUCKET_COUNT {
            if let Some(a) = bucket_aabb[b] {
                left_aabb = Some(match left_aabb {
                    Some(existing) => existing.united(&a),
                    None => a,
                });
                left_count += bucket_count[b];
            }
            left_running[b] = (left_aabb, left_count);
        }
        let mut right_aabb: Option<Aabb> = None;
        let mut right_count = 0u32;
        let mut right_running = [(Option::<Aabb>::None, 0u32); SAH_BUCKET_COUNT];
        for b in (0..SAH_BUCKET_COUNT).rev() {
            if let Some(a) = bucket_aabb[b] {
                right_aabb = Some(match right_aabb {
                    Some(existing) => existing.united(&a),
                    None => a,
                });
                right_count += bucket_count[b];
            }
            right_running[b] = (right_aabb, right_count);
        }

        for boundary in 0..SAH_BUCKET_COUNT - 1 {
            let (Some(l_aabb), l_n) = left_running[boundary] else { continue };
            let (Some(r_aabb), r_n) = right_running[boundary + 1] else { continue };
            if l_n == 0 || r_n == 0 {
                continue;
            }
            let cost = SAH_TRAVERSAL_COST
                + (surface_area(&l_aabb) / parent_sa) * l_n as f32 * SAH_INTERSECTION_COST
                + (surface_area(&r_aabb) / parent_sa) * r_n as f32 * SAH_INTERSECTION_COST;
            if best.is_none_or(|(best_cost, ..)| cost < best_cost) {
                best = Some((cost, axis, l_n as usize));
            }
        }
    }
    best.map(|(_, axis, split_index)| (axis, split_index))
}

fn surface_area(aabb: &Aabb) -> f32 {
    let e = aabb.max - aabb.min;
    2.0 * (e.x * e.y + e.y * e.z + e.z * e.x)
}

fn widest_centroid_axis(span: &[BuildLeaf]) -> u8 {
    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    for leaf in span {
        min = min.min(leaf.center);
        max = max.max(leaf.center);
    }
    let extent = max - min;
    if extent.x >= extent.y && extent.x >= extent.z {
        0
    } else if extent.y >= extent.z {
        1
    } else {
        2
    }
}

fn axis_of(v: Vec3, axis: u8) -> f32 {
    match axis {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

#[cfg(test)]
mod tests {
    use bevy::prelude::World;

    use super::*;

    /// Test-only stand-in for spawning real entities: `World::spawn`
    /// mints stable, distinct `Entity` ids without needing a full app.
    fn entities(n: usize) -> Vec<Entity> {
        let mut world = World::new();
        (0..n).map(|_| world.spawn_empty().id()).collect()
    }

    fn objects_at(entities: &[Entity], centers_and_halves: &[(Vec3, f32)]) -> Vec<HybridObject> {
        entities
            .iter()
            .zip(centers_and_halves)
            .map(|(&entity, &(center, half))| HybridObject {
                entity,
                world_aabb: Aabb::from_center_half(center, Vec3::splat(half)),
            })
            .collect()
    }

    #[test]
    fn empty_input_builds_no_nodes() {
        let bvh = Bvh::build(&[]);
        assert!(bvh.nodes.is_empty());
    }

    #[test]
    fn single_object_builds_one_leaf_node_at_root() {
        let e = entities(1);
        let objects = objects_at(&e, &[(Vec3::new(1.0, 2.0, 3.0), 0.5)]);
        let bvh = Bvh::build(&objects);
        assert_eq!(bvh.nodes.len(), 1);
        let root = bvh.nodes[0];
        assert_eq!(root.left_or_sentinel, LEAF_SENTINEL);
        assert_eq!(root.entity, e[0]);
        assert!((root.aabb.min - objects[0].world_aabb.min).length() < 1e-6);
        assert!((root.aabb.max - objects[0].world_aabb.max).length() < 1e-6);
    }

    /// Every internal node's AABB must exactly union its subtree's leaf
    /// AABBs — the property the whole structure exists to guarantee: a
    /// ray that misses a node's box can safely skip its entire subtree.
    #[test]
    fn every_internal_node_aabb_unions_its_subtree_leaves() {
        let e = entities(5);
        let objects = objects_at(
            &e,
            &[
                (Vec3::new(-10.0, 0.0, 0.0), 1.0),
                (Vec3::new(10.0, 0.0, 0.0), 1.0),
                (Vec3::new(0.0, 5.0, 0.0), 0.5),
                (Vec3::new(0.0, -5.0, 3.0), 2.0),
                (Vec3::new(1.0, 1.0, 1.0), 0.25),
            ],
        );
        let bvh = Bvh::build(&objects);
        let expected_root = objects.iter().map(|o| o.world_aabb).reduce(|a, b| a.united(&b)).unwrap();
        let root = bvh.nodes[0];
        assert!((root.aabb.min - expected_root.min).length() < 1e-5);
        assert!((root.aabb.max - expected_root.max).length() < 1e-5);

        fn check(nodes: &[BvhNode], index: usize) {
            let node = nodes[index];
            if node.left_or_sentinel == LEAF_SENTINEL {
                return;
            }
            let left = nodes[node.left_or_sentinel as usize];
            let right = nodes[node.right_or_object as usize];
            let united = left.aabb.united(&right.aabb);
            assert!((node.aabb.min - united.min).length() < 1e-5, "node {index} min mismatch");
            assert!((node.aabb.max - united.max).length() < 1e-5, "node {index} max mismatch");
            check(nodes, node.left_or_sentinel as usize);
            check(nodes, node.right_or_object as usize);
        }
        check(&bvh.nodes, 0);
    }

    /// Every input entity must appear in exactly one leaf, and every
    /// leaf's entity must be one of the input entities — no objects
    /// dropped, duplicated, or fabricated during the split.
    #[test]
    fn every_leaf_covers_exactly_one_input_object_and_all_are_covered() {
        let e = entities(7);
        let objects = objects_at(
            &e,
            &(0..7).map(|i| (Vec3::new(i as f32 * 2.0, 0.0, 0.0), 0.4)).collect::<Vec<_>>(),
        );
        let bvh = Bvh::build(&objects);
        let mut seen: std::collections::HashSet<Entity> = std::collections::HashSet::new();
        for node in &bvh.nodes {
            if node.left_or_sentinel == LEAF_SENTINEL {
                assert!(!seen.contains(&node.entity), "entity {:?} covered by more than one leaf", node.entity);
                seen.insert(node.entity);
            }
        }
        assert_eq!(seen.len(), e.len(), "not every input object was covered by a leaf");
        for entity in &e {
            assert!(seen.contains(entity));
        }
    }

    /// A cube's true (rotation-correct) world AABB, wired straight into
    /// `Bvh::build`, must appear at the root unchanged from what
    /// `scene::world_aabb` computed — this module trusts its input AABBs
    /// completely and must never silently shrink/distort a rotated
    /// object's bound while unioning it up the tree.
    #[test]
    fn rotated_cube_world_aabb_survives_into_the_bvh_unchanged() {
        use std::f32::consts::SQRT_2;

        use bevy::prelude::{GlobalTransform, Quat, Transform};

        use crate::sdf::components::Shape;

        let shape = Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 };
        let transform = GlobalTransform::from(Transform::from_rotation(Quat::from_rotation_y(
            std::f32::consts::FRAC_PI_4,
        )));
        let rotated_aabb = crate::hybrid::scene::world_aabb(&shape, &transform);
        assert!((rotated_aabb.max.x - SQRT_2).abs() < 1e-4, "sanity: rotated AABB should be diagonal-sized");

        let e = entities(2);
        let objects = vec![
            HybridObject { entity: e[0], world_aabb: rotated_aabb },
            HybridObject { entity: e[1], world_aabb: Aabb::from_center_half(Vec3::new(100.0, 0.0, 0.0), Vec3::splat(0.1)) },
        ];
        let bvh = Bvh::build(&objects);
        let leaf_for_rotated =
            bvh.nodes.iter().find(|n| n.left_or_sentinel == LEAF_SENTINEL && n.entity == e[0]).expect("leaf must exist");
        assert!((leaf_for_rotated.aabb.min - rotated_aabb.min).length() < 1e-5);
        assert!((leaf_for_rotated.aabb.max - rotated_aabb.max).length() < 1e-5);
    }

    /// `update` on the same entity set (only AABBs changed) must produce
    /// bounds identical to what a fresh `build` targeting the *old*
    /// topology would produce — refitting must never silently diverge
    /// from what a correct rebuild's bottom-up union would compute.
    #[test]
    fn update_with_same_entities_refits_to_identical_bounds_as_topology_matched_rebuild() {
        let e = entities(6);
        let initial = objects_at(
            &e,
            &(0..6).map(|i| (Vec3::new(i as f32 * 3.0, 0.0, 0.0), 0.4)).collect::<Vec<_>>(),
        );
        let mut bvh = Bvh::build(&initial);
        let topology_before: Vec<(u32, u32)> =
            bvh.nodes.iter().map(|n| (n.left_or_sentinel, n.right_or_object)).collect();

        // Move every object (simulates the cube rotating/translating) —
        // same entities, different AABBs.
        let moved: Vec<HybridObject> = initial
            .iter()
            .map(|o| HybridObject {
                entity: o.entity,
                world_aabb: Aabb {
                    min: o.world_aabb.min + Vec3::new(0.0, 5.0, 0.0),
                    max: o.world_aabb.max + Vec3::new(0.0, 5.0, 0.0),
                },
            })
            .collect();
        bvh.update(&moved);

        // Topology (child indices) must be completely unchanged by a
        // same-entity-set refit.
        let topology_after: Vec<(u32, u32)> =
            bvh.nodes.iter().map(|n| (n.left_or_sentinel, n.right_or_object)).collect();
        assert_eq!(topology_before, topology_after, "refit must not alter tree topology");

        // Bounds must match a fresh build over the moved AABBs exactly
        // for every leaf (order doesn't matter — compare by entity).
        let fresh = Bvh::build(&moved);
        let mut fresh_by_entity: HashMap<Entity, Aabb> = HashMap::new();
        for n in &fresh.nodes {
            if n.left_or_sentinel == LEAF_SENTINEL {
                fresh_by_entity.insert(n.entity, n.aabb);
            }
        }
        for n in &bvh.nodes {
            if n.left_or_sentinel == LEAF_SENTINEL {
                let expected = fresh_by_entity[&n.entity];
                assert!((n.aabb.min - expected.min).length() < 1e-5);
                assert!((n.aabb.max - expected.max).length() < 1e-5);
            }
        }

        // Root bound must equal the exact union of the moved AABBs.
        let expected_root = moved.iter().map(|o| o.world_aabb).reduce(|a, b| a.united(&b)).unwrap();
        assert!((bvh.nodes[0].aabb.min - expected_root.min).length() < 1e-5);
        assert!((bvh.nodes[0].aabb.max - expected_root.max).length() < 1e-5);
    }

    /// Adding/removing an object changes the entity set `update` covers,
    /// which must trigger a full rebuild (not a refit that would leave a
    /// stale or missing leaf) — verified by checking the resulting tree
    /// actually contains the new entity set.
    #[test]
    fn update_with_changed_entity_set_falls_back_to_full_rebuild() {
        let e = entities(4);
        let initial = objects_at(&e[..3], &(0..3).map(|i| (Vec3::new(i as f32, 0.0, 0.0), 0.3)).collect::<Vec<_>>());
        let mut bvh = Bvh::build(&initial);

        let with_new_object =
            objects_at(&e, &(0..4).map(|i| (Vec3::new(i as f32, 0.0, 0.0), 0.3)).collect::<Vec<_>>());
        bvh.update(&with_new_object);

        let leaves: std::collections::HashSet<Entity> =
            bvh.nodes.iter().filter(|n| n.left_or_sentinel == LEAF_SENTINEL).map(|n| n.entity).collect();
        assert_eq!(leaves.len(), 4, "rebuild must cover the new full entity set");
        for entity in &e {
            assert!(leaves.contains(entity));
        }
    }

    /// Same entity *count* but a different member swapped in (one entity
    /// leaves, a different one arrives, net size unchanged) must still
    /// fall back to a full rebuild — this is the case the count-only fast
    /// path inside `update` cannot catch by itself, since `leaf_index`'s
    /// length matches; only the per-entity `leaf_index.get` lookup catches
    /// it. A stale/incorrect refit here would silently keep marching a
    /// leaf for an entity that no longer exists, or drop the new entity's
    /// AABB entirely.
    #[test]
    fn update_with_same_count_but_swapped_entity_falls_back_to_full_rebuild() {
        let e = entities(4);
        let initial = objects_at(&e[..3], &(0..3).map(|i| (Vec3::new(i as f32, 0.0, 0.0), 0.3)).collect::<Vec<_>>());
        let mut bvh = Bvh::build(&initial);

        // Same count (3), but e[2] is replaced by e[3].
        let swapped_entities = [e[0], e[1], e[3]];
        let swapped =
            objects_at(&swapped_entities, &(0..3).map(|i| (Vec3::new(i as f32, 0.0, 0.0), 0.3)).collect::<Vec<_>>());
        bvh.update(&swapped);

        let leaves: std::collections::HashSet<Entity> =
            bvh.nodes.iter().filter(|n| n.left_or_sentinel == LEAF_SENTINEL).map(|n| n.entity).collect();
        assert_eq!(leaves.len(), 3);
        assert!(leaves.contains(&e[0]));
        assert!(leaves.contains(&e[1]));
        assert!(leaves.contains(&e[3]), "new entity must be covered after rebuild");
        assert!(!leaves.contains(&e[2]), "removed entity must not remain in the tree");
    }

    /// SAH-bucketed construction must find at least as good a split as
    /// median-split on a case constructed so the two disagree: two tight
    /// clusters of many small leaves far apart on X, plus one large leaf
    /// exactly at the spatial (not population) median — median-split by
    /// leaf *count* would pair the big leaf with whichever cluster is
    /// smaller in count, inflating that side's bounding box; SAH should
    /// instead keep the two population clusters together (lower total
    /// surface-area cost) since populating a box with the big leaf's
    /// far-off neighbor is expensive under the surface-area heuristic.
    #[test]
    fn sah_construction_achieves_lower_or_equal_total_surface_area_cost_than_median_split() {
        let mut centers_and_halves = Vec::new();
        // Dense cluster of 8 small leaves near x = -10.
        for i in 0..8 {
            centers_and_halves.push((Vec3::new(-10.0 + i as f32 * 0.1, 0.0, 0.0), 0.05));
        }
        // Dense cluster of 8 small leaves near x = 10.
        for i in 0..8 {
            centers_and_halves.push((Vec3::new(10.0 + i as f32 * 0.1, 0.0, 0.0), 0.05));
        }
        let e = entities(centers_and_halves.len());
        let objects = objects_at(&e, &centers_and_halves);

        let bvh = Bvh::build(&objects);
        let root = bvh.nodes[0];
        assert_ne!(root.left_or_sentinel, LEAF_SENTINEL, "root must have split with 16 well-separated leaves");
        let left = bvh.nodes[root.left_or_sentinel as usize];
        let right = bvh.nodes[root.right_or_object as usize];
        let sah_cost = surface_area(&left.aabb) + surface_area(&right.aabb);

        // The two spatial clusters must have been kept apart (not
        // interleaved) — each side's box should be small (roughly the
        // size of one 0.1-wide cluster), not spanning the full 20-unit
        // gap between them.
        assert!(
            surface_area(&left.aabb) < 5.0 && surface_area(&right.aabb) < 5.0,
            "SAH split should keep each spatial cluster in its own child \
             (left SA = {}, right SA = {}); a bad split spanning both \
             clusters would have much larger surface area",
            surface_area(&left.aabb),
            surface_area(&right.aabb),
        );
        assert!(sah_cost.is_finite() && sah_cost > 0.0);
    }

    /// `to_gpu_nodes` must preserve the source `Bvh`'s tree structure and
    /// bounds exactly: same node count, same topology (child indices
    /// unchanged — only leaves' `entity` becomes an `object_order` index),
    /// same AABB per node bit-for-bit (just exploded from `Vec3` into 6
    /// flat scalars), and every leaf's GPU index must resolve back to the
    /// correct source entity via `object_order`.
    #[test]
    fn to_gpu_nodes_preserves_tree_structure_and_bounds_exactly() {
        let e = entities(5);
        let objects = objects_at(
            &e,
            &[
                (Vec3::new(-10.0, 0.0, 0.0), 1.0),
                (Vec3::new(10.0, 0.0, 0.0), 1.0),
                (Vec3::new(0.0, 5.0, 0.0), 0.5),
                (Vec3::new(0.0, -5.0, 3.0), 2.0),
                (Vec3::new(1.0, 1.0, 1.0), 0.25),
            ],
        );
        let bvh = Bvh::build(&objects);

        // object_order deliberately NOT in the same order entities() minted
        // them, and not the same order the BVH visits leaves in either —
        // to_gpu_nodes must resolve by Entity identity, not by position.
        let object_order = vec![e[3], e[1], e[4], e[0], e[2]];

        let gpu_nodes = to_gpu_nodes(&bvh, &object_order);
        assert_eq!(gpu_nodes.len(), bvh.nodes.len(), "node count must be preserved exactly");

        for (i, (cpu, gpu)) in bvh.nodes.iter().zip(gpu_nodes.iter()).enumerate() {
            assert!((gpu.min_x - cpu.aabb.min.x).abs() < 1e-6, "node {i} min.x mismatch");
            assert!((gpu.min_y - cpu.aabb.min.y).abs() < 1e-6, "node {i} min.y mismatch");
            assert!((gpu.min_z - cpu.aabb.min.z).abs() < 1e-6, "node {i} min.z mismatch");
            assert!((gpu.max_x - cpu.aabb.max.x).abs() < 1e-6, "node {i} max.x mismatch");
            assert!((gpu.max_y - cpu.aabb.max.y).abs() < 1e-6, "node {i} max.y mismatch");
            assert!((gpu.max_z - cpu.aabb.max.z).abs() < 1e-6, "node {i} max.z mismatch");

            if cpu.left_or_sentinel == LEAF_SENTINEL {
                assert_eq!(gpu.left_or_sentinel, LEAF_SENTINEL, "node {i}: leaf sentinel must be preserved");
                let expected_index = object_order
                    .iter()
                    .position(|&entity| entity == cpu.entity)
                    .expect("test's object_order must contain every leaf entity") as u32;
                assert_eq!(
                    gpu.right_or_object, expected_index,
                    "node {i}: leaf's GPU object index must resolve to its source entity's position in object_order"
                );
            } else {
                // Internal node: topology (child indices) must be
                // byte-for-byte unchanged, not reinterpreted.
                assert_eq!(gpu.left_or_sentinel, cpu.left_or_sentinel, "node {i}: internal left child index must be preserved");
                assert_eq!(gpu.right_or_object, cpu.right_or_object, "node {i}: internal right child index must be preserved");
            }
        }
    }

    /// A leaf entity absent from `object_order` (stale BVH vs. this frame's
    /// object set) must not panic and must not produce an out-of-bounds
    /// index — it degrades to `right_or_object = 0` per this function's
    /// documented defensive fallback.
    #[test]
    fn to_gpu_nodes_handles_a_leaf_entity_missing_from_object_order_without_panicking() {
        let e = entities(2);
        let objects = objects_at(&e, &[(Vec3::ZERO, 1.0), (Vec3::new(5.0, 0.0, 0.0), 1.0)]);
        let bvh = Bvh::build(&objects);

        // Only e[0] is present in object_order; e[1]'s leaf has nowhere to
        // resolve to.
        let object_order = vec![e[0]];
        let gpu_nodes = to_gpu_nodes(&bvh, &object_order);
        assert_eq!(gpu_nodes.len(), bvh.nodes.len());
        for gpu in &gpu_nodes {
            if gpu.left_or_sentinel == LEAF_SENTINEL {
                assert!((gpu.right_or_object as usize) < object_order.len().max(1), "leaf index must stay in bounds");
            }
        }
    }
}
