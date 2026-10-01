//! Folds an ECS entity hierarchy of `Shape`/`Blend`/`Transform` components (see
//! `sdf::components`) into the plain `Node` tree `crate::bake` already knows how to
//! bake — the bridge between ECS-authored/animated primitives and the unchanged
//! baking/streaming pipeline. Called once per bake (see `streaming.rs`'s bake-task
//! spawn site), not per frame — bake cost dominates assembly cost by many orders of
//! magnitude (see `bake::sample_surface`'s doc comment), so re-walking the hierarchy
//! fresh on every bake is negligible overhead in exchange for always reflecting each
//! primitive's current `GlobalTransform`.

#[cfg(test)]
use bevy::asset::AssetPlugin;
#[cfg(test)]
use bevy::ecs::system::RunSystemOnce;
use bevy::prelude::*;
#[cfg(test)]
use bevy::shader::Shader;

use super::components::{AnimGroup, Blend, BlendMode, MaterialLegacy, ProceduralPattern, Shape};
use super::scene::{Isometry, LeafMaterial, Node};

/// The 6-tuple `assemble_*`'s `shapes` parameter reads a `Shape` entity through,
/// used as `Query<ShapeQueryData>`. This aliases only the query *data* (with
/// `'static` component references, the form Bevy's `SystemParam` impl requires
/// anyway), never the `Query` itself: a lifetime-parameterized `Query`-wrapping
/// alias was tried and hit `Query<'w, 's, D>`'s invariance over `D` (a closure
/// passed to `run_system_once_with` in `sdf::world`'s tests, whose `'w`/`'s` are
/// inferred by Bevy's own machinery, didn't reliably unify with it), whereas
/// leaving `'w`/`'s` elided at each use site sidesteps that entirely.
pub type ShapeQueryData = (
    &'static Shape,
    &'static GlobalTransform,
    Option<&'static BlendMode>,
    Option<&'static Blend>,
    Option<&'static MaterialLegacy>,
    Option<&'static ProceduralPattern>,
);

/// Resolves an entity's `MaterialLegacy`/`ProceduralPattern` components (both optional) into
/// a `LeafMaterial` for `Node::leaf_boxed_with_material` — `None` if neither component
/// is present (see `Node::Leaf::material`'s doc comment for the fallback that produces
/// downstream). A `ProceduralPattern` with no `MaterialLegacy` sibling has no "material A" to
/// hand its pattern function; rather than silently defaulting material A to something
/// arbitrary, this falls back to treating the entity as `Solid`-materialless in that
/// case too — an entity wanting a pattern must author both components, matching
/// `ProceduralPattern`'s own doc comment ("required alongside `ProceduralPattern`").
pub(crate) fn resolve_leaf_material(
    material: Option<&MaterialLegacy>,
    pattern: Option<&ProceduralPattern>,
) -> Option<LeafMaterial> {
    match (material, pattern) {
        (Some(mat), Some(pattern)) => Some(LeafMaterial::Pattern {
            import_path: pattern.import_path.clone(),
            material_a: (mat.base_color, mat.metallic, mat.roughness),
            material_b: (
                pattern.material_b.base_color,
                pattern.material_b.metallic,
                pattern.material_b.roughness,
            ),
            params: pattern.params,
        }),
        (Some(mat), None) => Some(LeafMaterial::Solid {
            base_color: mat.base_color,
            metallic: mat.metallic,
            roughness: mat.roughness,
        }),
        (None, _) => None,
    }
}

/// Default smooth-union blend radius for a child entity with `Shape` but no explicit
/// `BlendMode` or `Blend` component — matches `sdf::world`'s old `JOIN_RADIUS` (the
/// CSG-join radius used for every top-level object join in the hand-written scene,
/// see that constant's doc comment for why a nonzero default matters: a hard boolean
/// seam is a gradient discontinuity that destabilizes `bake::principal_curvature_frame`'s
/// curvature estimate the same way a sharp primitive corner does).
pub const DEFAULT_BLEND: f32 = 0.08;

/// Resolves an entity's optional `BlendMode`/`Blend` components into an effective
/// `BlendMode` — `BlendMode` takes priority; legacy `Blend(f32)` maps to
/// `SmoothUnion(k)`; neither present falls back to `SmoothUnion(DEFAULT_BLEND)`.
fn resolve_blend_mode(blend_mode: Option<&BlendMode>, legacy_blend: Option<&Blend>) -> BlendMode {
    match (blend_mode, legacy_blend) {
        (Some(mode), _) => *mode,
        (None, Some(blend)) => BlendMode::SmoothUnion(blend.0),
        (None, None) => BlendMode::SmoothUnion(DEFAULT_BLEND),
    }
}

/// Applies a CSG blend operation between two `Node`s according to the resolved
/// `BlendMode` — the single dispatch point for all CSG operations, replacing the
/// old always-smooth-union logic.
fn apply_blend(parent: Node, child: Node, mode: BlendMode) -> Node {
    match mode {
        BlendMode::Union => parent.union(child),
        BlendMode::SmoothUnion(k) => parent.smooth_union(child, k),
        BlendMode::Intersect => parent.intersect(child),
        BlendMode::SmoothIntersect(k) => parent.smooth_intersect(child, k),
        BlendMode::Subtract(k) => parent.smooth_subtract(child, k),
    }
}

/// Marks the root entity of one independently-assembled SDF scene — `assemble_scene`
/// walks this entity's `Children` (recursively) to build the `Node` tree. Multiple
/// roots can coexist (e.g. the opaque tile cluster and the translucent demo sphere
/// are separate scenes, per `sdf::world::translucent_scene`'s doc comment on why they
/// bake separately) — each is assembled independently by calling `assemble_scene`
/// with that root's `Entity`.
#[derive(Component, Clone, Copy, Debug)]
pub struct SdfSceneRoot;

/// Builds the `Node` tree for the subtree rooted at `root`, reading each entity's
/// `Shape` (if any) and `GlobalTransform`, and combining children together (and with
/// their parent's own `Shape`, if it has one) via the CSG operation specified by each
/// child's `BlendMode` component (or legacy `Blend(f32)`, or `DEFAULT_BLEND` if
/// neither is present).
///
/// Returns `None` for an empty subtree (an entity with no `Shape` and no children with
/// a `Shape`) — a caller combining several assembled roots together (see
/// `sdf::world::tile_cluster`'s ECS-authored replacement) should skip `None` results
/// rather than trying to smooth_union with an empty node, since there's no sensible
/// "distance to nothing" to blend against.
pub fn assemble_scene(
    root: Entity,
    shapes: &Query<ShapeQueryData>,
    children_of: &Query<&Children>,
) -> Option<Node> {
    let own = shapes
        .get(root)
        .ok()
        .map(|(shape, transform, _, _, material, pattern)| {
            Node::leaf_boxed(shape.sdf(), isometry_from_global_transform(transform))
                .with_material(resolve_leaf_material(material, pattern))
        });

    let mut accumulated = own;

    if let Ok(children) = children_of.get(root) {
        for &child in children {
            let Some(child_node) = assemble_scene(child, shapes, children_of) else {
                continue;
            };
            let blend_mode = shapes
                .get(child)
                .ok()
                .map(|(_, _, blend_mode, legacy_blend, ..)| {
                    resolve_blend_mode(blend_mode, legacy_blend)
                })
                .unwrap_or_default();

            accumulated = Some(match accumulated {
                Some(acc) => apply_blend(acc, child_node, blend_mode),
                None => child_node,
            });
        }
    }

    accumulated
}

/// `GlobalTransform` carries scale too (a full affine matrix), but `Isometry` is
/// deliberately translation+rotation only (see that type's doc comment on why
/// non-uniform SDF scale isn't supported) — this extracts just the rotation and
/// translation, silently ignoring scale rather than erroring, since a `Transform`
/// authored with scale != 1 on an SDF primitive entity is a misuse this demo doesn't
/// need to guard against yet (no such entity exists in the current scene).
fn isometry_from_global_transform(transform: &GlobalTransform) -> Isometry {
    let (_scale, rotation, translation) = transform.to_scale_rotation_translation();
    Isometry {
        translation,
        rotation,
    }
}

/// One `AnimGroup`'s rest-pose bake input: its own subtree assembled with rotation
/// stripped back to identity (translation kept), plus the pivot (the group entity's
/// live world-space translation) the render pipeline needs to reapply the group's
/// *live* rotation around, every frame, on the GPU — see `AnimGroup`'s doc comment.
pub struct AnimGroupScene {
    pub id: u32,
    pub node: Node,
    pub pivot: Vec3,
}

/// Like `assemble_scene`, but splits the hierarchy into the non-animated (static)
/// subtree and one `AnimGroupScene` per `AnimGroup` entity found — the two-bake-target
/// split `deformation-vs-rebake.md`'s rigid-transform rule requires (see `AnimGroup`'s
/// doc comment). An `AnimGroup` entity is *not* folded into the static result even
/// though it's still a descendant of `root` in the ECS hierarchy — its whole subtree is
/// instead assembled independently, rooted at identity rotation, with its own pivot
/// recorded separately.
///
/// This only supports one level of `AnimGroup` nesting (an `AnimGroup` entity's own
/// descendants are never themselves additional `AnimGroup`s) — true of every entity
/// `sdf::world::spawn_tile_cluster` spawns today (the pillar is a single leaf entity;
/// each ring is a pivot-only entity — no `Shape` of its own — with bead-sphere leaf
/// children), and nested anim groups aren't needed by anything this demo does yet.
///
/// `transforms` is a plain `GlobalTransform`-only query (not the `shapes` 6-tuple)
/// so an `AnimGroup` entity's pivot can be read even when that entity carries no
/// `Shape` itself — true of the ring entities, which are pure transform pivots for
/// their bead children (see `sdf::world::spawn_tile_cluster`'s ring-spawning loop).
pub fn assemble_scene_split(
    root: Entity,
    shapes: &Query<ShapeQueryData>,
    children_of: &Query<&Children>,
    anim_groups: &Query<&AnimGroup>,
    transforms: &Query<&GlobalTransform>,
) -> (Option<Node>, Vec<AnimGroupScene>) {
    let mut groups = Vec::new();
    let static_node = assemble_scene_excluding_groups(
        root,
        shapes,
        children_of,
        anim_groups,
        transforms,
        &mut groups,
    );
    (static_node, groups)
}

fn assemble_scene_excluding_groups(
    root: Entity,
    shapes: &Query<ShapeQueryData>,
    children_of: &Query<&Children>,
    anim_groups: &Query<&AnimGroup>,
    transforms: &Query<&GlobalTransform>,
    groups: &mut Vec<AnimGroupScene>,
) -> Option<Node> {
    let own = shapes
        .get(root)
        .ok()
        .map(|(shape, transform, _, _, material, pattern)| {
            Node::leaf_boxed(shape.sdf(), isometry_from_global_transform(transform))
                .with_material(resolve_leaf_material(material, pattern))
        });

    let mut accumulated = own;

    if let Ok(children) = children_of.get(root) {
        for &child in children {
            if let Ok(AnimGroup(id)) = anim_groups.get(child) {
                // This child is an anim-group root: assemble its own subtree
                // separately, in rest pose (rotation stripped), rather than folding it
                // into `accumulated` — see `assemble_scene_split`'s doc comment. The
                // pivot comes from `transforms` (not `shapes`) since the group root
                // itself may carry no `Shape` (e.g. a ring's bead-sphere children do
                // the actual leaf work).
                if let Ok(transform) = transforms.get(child) {
                    let pivot = transform.translation();
                    if let Some(rest_pose) =
                        assemble_rest_pose_subtree(child, shapes, children_of, pivot)
                    {
                        groups.push(AnimGroupScene {
                            id: *id,
                            node: rest_pose,
                            pivot,
                        });
                    }
                }
                continue;
            }

            let Some(child_node) = assemble_scene_excluding_groups(
                child,
                shapes,
                children_of,
                anim_groups,
                transforms,
                groups,
            ) else {
                continue;
            };
            let blend_mode = shapes
                .get(child)
                .ok()
                .map(|(_, _, blend_mode, legacy_blend, ..)| {
                    resolve_blend_mode(blend_mode, legacy_blend)
                })
                .unwrap_or_default();

            accumulated = Some(match accumulated {
                Some(acc) => apply_blend(acc, child_node, blend_mode),
                None => child_node,
            });
        }
    }

    accumulated
}

/// Assembles `root`'s subtree exactly like `assemble_scene`, except every leaf's
/// isometry is expressed **relative to `pivot`, with `root`'s own rotation stripped to
/// identity** — the "rest pose" `AnimGroupScene` bakes from. Positioning relative to
/// `pivot` (not world space) is what lets the render pipeline reconstruct
/// `world_pos = pivot + live_rotation * splat.position` on the GPU every frame (see
/// `splat::asset::AnimGroupCloud`'s doc comment) — baking in world space would make
/// that reconstruction need to also know and subtract the pivot itself out of every
/// splat position at render time, needless extra per-splat GPU work when it's free to
/// do once at bake time instead. Descendants (none exist for any current `AnimGroup`
/// entity, but the recursion supports them) keep their normal `GlobalTransform`-
/// derived rotation and are shifted by the same `-pivot` translation as `root`.
fn assemble_rest_pose_subtree(
    root: Entity,
    shapes: &Query<ShapeQueryData>,
    children_of: &Query<&Children>,
    pivot: Vec3,
) -> Option<Node> {
    let own = shapes
        .get(root)
        .ok()
        .map(|(shape, transform, _, _, material, pattern)| {
            let mut isometry = isometry_from_global_transform(transform);
            isometry.translation -= pivot;
            isometry.rotation = Quat::IDENTITY;
            Node::leaf_boxed(shape.sdf(), isometry)
                .with_material(resolve_leaf_material(material, pattern))
        });

    let mut accumulated = own;

    if let Ok(children) = children_of.get(root) {
        for &child in children {
            let Some(child_node) = assemble_rest_pose_subtree(child, shapes, children_of, pivot)
            else {
                continue;
            };
            let blend_mode = shapes
                .get(child)
                .ok()
                .map(|(_, _, blend_mode, legacy_blend, ..)| {
                    resolve_blend_mode(blend_mode, legacy_blend)
                })
                .unwrap_or_default();

            accumulated = Some(match accumulated {
                Some(acc) => apply_blend(acc, child_node, blend_mode),
                None => child_node,
            });
        }
    }

    accumulated
}

/// Test-only helper: builds a headless `World`, runs `spawn` as a one-shot system via
/// `World::run_system_once` (so it gets real `Commands`), then computes
/// `GlobalTransform` for every spawned entity by hand rather than via Bevy's own
/// `propagate_parent_transforms` system — that system calls `ComputeTaskPool::get()`
/// internally on this Bevy build (the `multi_threaded` feature is enabled, see
/// bevy_transform's systems.rs), which needs full `App`/`TaskPoolPlugin`/scheduler
/// context to run without panicking on a background worker thread; not worth fighting
/// for a test helper.
///
/// The hierarchies `spawn_tile_cluster`-style functions build can now be THREE levels
/// deep (root -> ring-pivot child with no `Shape` of its own -> rotated bead-sphere
/// grandchildren — see `sdf::world::spawn_tile_cluster`'s ring-spawning loop), so this
/// composes each entity's world transform from its own `Transform` multiplied through
/// every ancestor's `Transform` up to a root with no `ChildOf`, via `Transform::mul_transform`
/// (breadth-first from the roots so every parent's `GlobalTransform` is already resolved
/// before its children are computed) — a flat per-entity `GlobalTransform::from(*transform)`
/// would silently drop the ring pivot's rotation+translation from its beads.
/// `spawn` takes `&AssetServer` too (not just `&mut Commands`) since `spawn_tile_cluster`
/// itself now loads the checkerboard pattern shader by asset path — a bare
/// `World::new()` has no `AssetServer` to hand it, so this builds a minimal `App` with
/// just `AssetPlugin` (not the full `DefaultPlugins`/rendering stack this test helper
/// otherwise deliberately avoids, see below) and pulls its `World` out instead.
#[cfg(test)]
pub fn test_app_with(
    spawn: impl FnOnce(&mut Commands, &AssetServer) -> Entity + Send + Sync + 'static,
) -> (World, Entity) {
    let mut app = App::new();
    // TaskPoolPlugin (not the full DefaultPlugins/rendering stack this test helper
    // otherwise deliberately avoids) is the lightweight, headless-safe plugin that
    // initializes IoTaskPool — AssetServer::load (which spawn_tile_cluster now calls,
    // to load shaders/patterns/checkerboard.wgsl) panics without it.
    app.add_plugins((bevy::app::TaskPoolPlugin::default(), AssetPlugin::default()));
    // AssetPlugin alone doesn't register the `Shader` asset type (only
    // `bevy_render`'s own plugin does, via `init_asset::<Shader>()` — pulling that
    // whole plugin in for this test helper would mean real GPU device
    // initialization, which doesn't work headless), so register just the asset type
    // directly.
    app.init_asset::<Shader>();
    let mut spawn = Some(spawn);
    let root = app
        .world_mut()
        .run_system_once(
            move |mut commands: Commands, asset_server: Res<AssetServer>| {
                (spawn.take().unwrap())(&mut commands, &asset_server)
            },
        )
        .unwrap();
    let mut world = std::mem::take(app.world_mut());

    // Breadth-first from every root (`ChildOf`-less entity with a `Transform`) so
    // each parent's resolved `GlobalTransform` is available before its children
    // are computed.
    let mut query = world.query::<(Entity, &Transform, Option<&ChildOf>)>();
    let mut local: bevy::platform::collections::HashMap<Entity, Transform> = Default::default();
    let mut roots = Vec::new();
    for (entity, transform, child_of) in query.iter(&world) {
        local.insert(entity, *transform);
        if child_of.is_none() {
            roots.push(entity);
        }
    }

    let mut resolved: bevy::platform::collections::HashMap<Entity, GlobalTransform> =
        Default::default();
    let mut queue: std::collections::VecDeque<Entity> = roots.into();
    for &root in &queue {
        resolved.insert(root, GlobalTransform::from(local[&root]));
    }
    while let Some(entity) = queue.pop_front() {
        let Some(children) = world.get::<Children>(entity) else {
            continue;
        };
        let parent_global = resolved[&entity];
        let children: Vec<Entity> = children.iter().collect();
        for child in children {
            let child_global = parent_global.mul_transform(local[&child]);
            resolved.insert(child, child_global);
            queue.push_back(child);
        }
    }

    for (entity, global_transform) in resolved {
        world.entity_mut(entity).insert(global_transform);
    }

    (world, root)
}
