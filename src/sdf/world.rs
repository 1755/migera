//! The demo world: an ECS-authored SDF scene (see `sdf::components`/`sdf::assembly`)
//! exercising union, smooth union, and subtraction, per
//! docs/knowledge/sdf-3d/primitives-and-operators/combination-operators.md.
//!
//! Primitives are ordinary Bevy entities (`Transform` + `Shape`, grouped via
//! `ChildOf`/`Children`) rather than a hand-written `Node` tree — see
//! `spawn_tile_cluster` and `assembly::assemble_scene`. This is what makes animating a
//! primitive (see `main.rs`'s animation system) literal `Transform` mutation: the next
//! bake's `assemble_scene` call reads whatever `GlobalTransform` is current at that
//! moment, with no separate animation-to-SDF bridge needed.

use bevy::prelude::*;

use super::assembly::{self, SdfSceneRoot, ShapeQueryData};
use super::components::{AnimGroup, BlendMode, MaterialLegacy, ProceduralPattern, Shape};
use super::scene::Node;

/// The minimum fillet/edge radius any hard-edged primitive in this scene uses,
/// rounding every sharp corner/rim just enough for `bake`'s curvature estimation to
/// stay numerically stable there — see `RoundedBox3`/`RoundedCylinder`'s doc comments
/// for the mechanism. Must be comfortably larger than `bake::BakeSettings::gradient_eps`
/// (the finite-difference tap distance `principal_curvature_frame` uses): a fillet
/// radius close to or smaller than that tap distance still looks like a near-sharp
/// corner to the curvature stencil (a tap can land just past the fillet's arc, back
/// into the region where curvature is still large/discontinuous), which is why the
/// ground's first rounding attempt (`corner_radius: 0.03`, equal to `gradient_eps`)
/// closed the coverage *gap* at the edge but still produced visibly mis-oriented
/// "winged" splats right at the rim — the curvature estimate there was still noisy,
/// not the fillet itself being too small to render smoothly. This value is roughly
/// 3x `gradient_eps` (0.03), giving the tangent-plane taps room to land within the
/// smoothly-curved fillet arc instead of straddling back out of it.
const EDGE_RADIUS: f32 = 0.1;

/// Blend radius for every CSG join in the scene, not just individual primitives' own
/// corners — a hard boolean seam between two otherwise-smooth shapes is exactly as
/// much a gradient discontinuity as a sharp primitive corner, and destabilizes
/// `bake::principal_curvature_frame`'s curvature estimate the same way (see
/// `EDGE_RADIUS`'s doc comment for the mechanism). The pillar's own bite-subtraction
/// seam and the top-level ground/blob/arch/pillar joins all showed the same "winged"
/// splat artifact right at their contact seams even after every individual
/// primitive's own edges were rounded — this is what closes that remaining gap. Kept
/// small enough that a hard contact still reads as a hard contact visually (this is a
/// numerical-stability fix, not an intentional aesthetic bevel). Matches
/// `assembly::DEFAULT_BLEND` — every join in this scene uses the default, so no
/// entity needs an explicit `Blend` override.
const JOIN_RADIUS: f32 = assembly::DEFAULT_BLEND;

/// Side length of one repeat cell in `infinite_scene()`'s domain-repeated grid (see
/// `Node::Repeat`/`repeat_xz`). Sized comfortably larger than the tile cluster's own
/// footprint (its objects span roughly x in [-4.2, 5.8], z in [-1, 1], and its ground
/// tile is clipped to a 6.5-unit half-extent = 13-unit-wide footprint) so neighboring
/// copies never reach across a cell boundary — the domain-repetition correctness
/// caveat from docs/knowledge/sdf-3d/primitives-and-operators/domain-operations.md
/// ("naive mod()-based repetition is only exact/safe when neighboring cells' shapes
/// cannot reach into an adjacent cell"). 16 gives ~1.5 units of clearance on each side
/// beyond the ground tile's own 13-unit footprint.
pub const TILE_PERIOD: f32 = 16.0;

/// Marker for the pillar entity `spawn_tile_cluster` creates — `main.rs`'s animation
/// system queries this to find the entity whose `Transform` it should rotate each
/// frame, without needing to know the rest of the hierarchy `spawn_tile_cluster`
/// builds internally.
#[derive(Component)]
pub struct AnimatedPillar;

/// The tile cluster's `SdfSceneRoot` entity, stashed as a resource once at startup
/// (see `main.rs`'s `setup`) so `streaming::update_chunk_streaming` can find it every
/// time it needs to assemble a fresh `Node` for a bake, without re-spawning the
/// cluster or threading the `Entity` through some other channel.
#[derive(Resource, Clone, Copy)]
pub struct TileClusterRoot(pub Entity);

/// Spawns one repeat cell's worth of scenery as an entity hierarchy under a fresh
/// `SdfSceneRoot` entity (returned), for `assembly::assemble_scene` to fold into a
/// `Node` tree at bake time:
/// - a ground plane clipped to a finite slab (a plane is infinite and would swallow
///   the whole bake volume, so a wide, thin rounded box stands in for it) — rounded at
///   the edges (`Shape::RoundedBox`, not a sharp box) so the baker sees a continuous,
///   numerically stable surface normal/curvature everywhere: a true hard edge is a
///   genuine gradient discontinuity that both starves splat sampling right at the
///   crease AND (more visibly) destabilizes the curvature-derived splat orientation
///   there.
/// - a smooth-blended "blob" cluster of spheres (organic shape via smooth union).
/// - a cylinder pillar (rounded top/bottom rim, same reasoning as the ground) with a
///   sphere bite smooth-subtracted out of it — the bite is modeled as a child
///   entity with `BlendMode::Subtract` (see `assembly::assemble_scene`'s handling of
///   `BlendMode::Subtract`), so it's assembled automatically by `assembly` instead of
///   requiring a one-off hand-written `smooth_subtract` call in `assemble_infinite_scene`;
///   the pillar's own cylinder IS ECS-authored (and thus `Transform`-animatable), the
///   bite sphere is a fixed child positioned relative to it.
pub fn spawn_tile_cluster(commands: &mut Commands, asset_server: &AssetServer) -> Entity {
    // The root needs its own Transform (even though assembly.rs never reads it —
    // SdfSceneRoot entities have no Shape of their own): a ChildOf-parented entity's
    // GlobalTransform does not propagate correctly if the parent itself lacks a
    // Transform/GlobalTransform — every child silently collapsed to world-origin
    // (GlobalTransform::translation() == Vec3::ZERO for all of them) without this,
    // which showed up as the whole tile cluster baking as one small blob (every
    // primitive overlapping at the same point) instead of the intended spread-out
    // scene.
    let root = commands
        .spawn((SdfSceneRoot, Transform::IDENTITY, Visibility::default()))
        .id();

    // Ground: a procedural checkerboard pattern (see assets/shaders/patterns/
    // checkerboard.wgsl — the reference implementation of the pattern-authoring
    // contract, loaded here exactly like a user-authored pattern shader would be, no
    // privileged access) with a polished, mirror-reflective dark cell and a rough
    // matte light cell — demonstrating both halves of the extensible-materials system
    // in one primitive: an authored `Material` ("material A") plus a dynamically
    // dispatched `ProceduralPattern` selecting between it and a second material.
    let checkerboard_shader = asset_server.load("shaders/patterns/checkerboard.wgsl");
    commands.spawn((
        ChildOf(root),
        Shape::RoundedBox {
            half_extents: Vec3::new(6.5, 0.15, 6.5),
            corner_radius: EDGE_RADIUS,
        },
        Transform::from_xyz(0.0, -0.15, 0.0),
        // Dark cell ("material A", see ProceduralPattern's doc comment): near-black,
        // low roughness, dielectric — polished dark stone/obsidian, reflective via
        // its own Fresnel response despite metallic=0 (dielectrics still reflect
        // strongly at grazing angles — see is_mirror_reflective/fresnel_schlick in
        // raymarch.wgsl).
        MaterialLegacy::new(Vec3::new(0.03, 0.03, 0.035), 0.0, 0.05),
        ProceduralPattern {
            shader: checkerboard_shader,
            import_path: "migera::pattern::checkerboard".to_string(),
            // Light cell ("material B"): matte off-white, high roughness —
            // deliberately NOT reflective (rough enough that is_mirror_reflective
            // excludes it from the traced-reflection path), so the chessboard reads
            // as "one cell type is a polished floor, the other is chalky stone"
            // rather than two colors of the same material.
            material_b: MaterialLegacy::new(Vec3::new(0.85, 0.83, 0.78), 0.0, 0.9),
            // params.x = checkerboard cell size in world units — see
            // patterns/checkerboard.wgsl's evaluate_pattern.
            params: Vec4::new(1.0, 0.0, 0.0, 0.0),
        },
    ));

    // Organic blob cluster: several spheres smooth-unioned together (via
    // assemble_scene folding all of the root's children together, per its doc
    // comment — these don't need their own sub-group entity since every join in this
    // scene uses the same DEFAULT_BLEND radius). Each sphere carries its own Material
    // — deliberately a mix of dielectric and metallic so the union visibly
    // demonstrates per-primitive material variety, smoothly blended at the CSG seams
    // (see raymarch.wgsl's blended_material_color) rather than a hard per-primitive
    // color switch.
    commands.spawn((
        ChildOf(root),
        Shape::Sphere { radius: 1.1 },
        Transform::from_xyz(-3.0, 1.4, 0.0),
        MaterialLegacy::new(Vec3::new(0.80, 0.24, 0.20), 0.0, 0.45), // terracotta clay (dielectric)
    ));
    commands.spawn((
        ChildOf(root),
        Shape::Sphere { radius: 0.8 },
        Transform::from_xyz(-2.2, 1.9, 0.6),
        MaterialLegacy::new(Vec3::new(0.90, 0.58, 0.20), 1.0, 0.25), // gold (metal)
    ));
    commands.spawn((
        ChildOf(root),
        Shape::Sphere { radius: 0.9 },
        Transform::from_xyz(-3.6, 2.1, -0.5),
        MaterialLegacy::new(Vec3::new(0.16, 0.55, 0.45), 0.0, 0.60), // jade (dielectric, rougher)
    ));
    commands.spawn((
        ChildOf(root),
        Shape::Sphere { radius: 0.5 },
        Transform::from_xyz(-2.6, 2.6, 0.0),
        MaterialLegacy::new(Vec3::new(0.75, 0.76, 0.78), 1.0, 0.12), // polished steel (metal, smooth)
    ));

    // Pillar (the bite is now a child entity with BlendMode::Subtract — see
    // assemble_infinite_scene). AnimatedPillar marks it for main.rs's rotation system;
    // AnimGroup(PILLAR_ANIM_GROUP) marks it for GPU-side rotation reapplication.
    let pillar = commands
        .spawn((
            ChildOf(root),
            AnimatedPillar,
            AnimGroup(PILLAR_ANIM_GROUP),
            Shape::RoundedCylinder {
                radius: 0.6,
                half_height: 1.6,
                edge_radius: EDGE_RADIUS,
            },
            Transform::from_xyz(4.2, 1.6, -1.0),
        ))
        .id();

    // Bite sphere: child of the pillar, positioned relative to it. BlendMode::Subtract
    // makes assembly apply smooth_subtract when folding the pillar's children — the
    // sphere carves a bite out of the pillar automatically, no manual post-assembly
    // patching needed. Offset (0.0, 0.8, 0.7) relative to pillar matches the old
    // scene-relative offset (4.2, 2.4, -0.3) at the pillar's authored position.
    commands.spawn((
        ChildOf(pillar),
        Shape::Sphere { radius: 0.9 },
        Transform::from_xyz(0.0, 0.8, 0.7),
        BlendMode::Subtract(JOIN_RADIUS),
    ));

    root
}

/// `AnimGroup` ID for the pillar (see `spawn_tile_cluster`). Kept as a named constant
/// rather than inlined `0` since `assemble_infinite_scene` needs to identify this
/// specific group (to apply the bite) among whatever else `assemble_scene_split`
/// returns.
const PILLAR_ANIM_GROUP: u32 = 0;

/// The result of assembling the tile cluster for one bake request: the (non-animated)
/// static geometry — already `Node::Repeat`-wrapped for infinite tiling, ready to bake
/// directly — plus one rest-pose `AnimGroupScene` per `AnimGroup` entity (just the
/// pillar), each ready to bake *separately* into its own small `SplatCloud` (see
/// `assembly::assemble_scene_split`'s doc comment and `AnimGroup`'s doc comment for why
/// animated content is split out rather than folded into the one big static bake).
pub struct TileClusterScene {
    pub static_scene: Node,
    pub anim_groups: Vec<assembly::AnimGroupScene>,
}

/// Folds `root`'s entity hierarchy into the static/anim-group split
/// `assembly::assemble_scene_split` produces, applies the pillar's bite to the
/// pillar's own rest-pose subtree specifically (now that the pillar bakes separately
/// in its own local frame, the bite's offset is relative to the pillar's *own* pivot,
/// not scene-relative — fixing the large-translation-detaches-the-bite limitation the
/// old single-assembled-cluster approach had, see the previous version of this
/// function's doc comment), and wraps the static geometry in `Node::Repeat` at
/// `TILE_PERIOD` spacing for infinite tiling, per
/// docs/knowledge/sdf-3d/primitives-and-operators/domain-operations.md. Anim-group
/// scenes are NOT repeat-wrapped here — they're baked as their own small, un-tiled
/// `SplatCloud`s and positioned per-instance by the render pipeline instead (see
/// `streaming.rs`'s bake-task spawn site).
///
/// Called on the main thread (needs `Query` access to the live ECS state) once per
/// bake request, then the resulting `Node`s — plain, `Send`-safe owned data, no
/// lifetime tied to the `World` — are moved into the background bake task (see
/// `streaming.rs`'s bake-task spawn site): assembly cost is negligible next to bake
/// cost (see `bake::sample_surface`'s doc comment), so re-walking the hierarchy fresh
/// per bake trades a trivial amount of main-thread time for always reflecting each
/// primitive's current (possibly animated) `Transform`.
pub fn assemble_infinite_scene(
    root: Entity,
    shapes: &Query<ShapeQueryData>,
    children_of: &Query<&Children>,
    anim_groups_query: &Query<&AnimGroup>,
    transforms: &Query<&GlobalTransform>,
) -> TileClusterScene {
    let (static_node, anim_groups) = assembly::assemble_scene_split(
        root,
        shapes,
        children_of,
        anim_groups_query,
        transforms,
    );
    let static_node = static_node
        .expect("tile cluster root should always have at least one non-animated Shape descendant");

    // The pillar's bite is now handled automatically by assembly: the bite sphere
    // is a child entity of the pillar with BlendMode::Subtract, so assembly applies
    // smooth_subtract when folding the pillar's children together — no manual
    // post-assembly patching needed anymore.

    TileClusterScene {
        static_scene: static_node.repeat(TILE_PERIOD),
        anim_groups,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::assembly::test_app_with;
    use bevy::ecs::system::RunSystemOnce;

    #[test]
    fn infinite_scene_repeats_the_blob_at_the_next_cell() {
        let (mut world, root) = test_app_with(spawn_tile_cluster);
        let scene = world.run_system_once_with(
            move |shapes: Query<ShapeQueryData>,
                  children_of: Query<&Children>,
                  anim_groups: Query<&AnimGroup>,
                  transforms: Query<&GlobalTransform>| {
                assemble_infinite_scene(root, &shapes, &children_of, &anim_groups, &transforms)
            },
            (),
        );
        let scene = scene.unwrap();

        // A point empirically confirmed on the smooth-unioned blob cluster's surface
        // within cell (0,0,0): world +X out from the largest sphere's center
        // (-3.0, 1.4, 0.0), radius 1.1 (see spawn_tile_cluster's blob-cluster spawn
        // loop) — but NOT at exactly radius 1.1, since this point sits on the
        // *smooth-unioned* surface, which the neighboring blob spheres' smooth_union
        // pulls slightly inward from the raw sphere's own surface (a few thousandths,
        // here) — same reasoning as the pre-AnimGroup-split ring surface point this
        // test's predecessor used to need (see this test's git history).
        let p0 = Vec3::new(-3.0 + 1.107, 1.4, 0.0);
        let p1 = p0 + Vec3::new(TILE_PERIOD, 0.0, 0.0);
        let d0 = scene.static_scene.distance(p0);
        let d1 = scene.static_scene.distance(p1);
        assert!(d0.abs() < 1e-3, "cell 0 blob point not on surface: {d0}");
        assert!(
            d1.abs() < 1e-3,
            "repeated cell blob point not on surface: {d1}"
        );
    }

    #[test]
    fn pillar_bakes_at_rest_pose_relative_to_its_own_pivot() {
        let (mut world, root) = test_app_with(spawn_tile_cluster);
        let scene = world
            .run_system_once_with(
                move |shapes: Query<ShapeQueryData>,
                      children_of: Query<&Children>,
                      anim_groups: Query<&AnimGroup>,
                      transforms: Query<&GlobalTransform>| {
                    assemble_infinite_scene(root, &shapes, &children_of, &anim_groups, &transforms)
                },
                (),
            )
            .unwrap();

        // Pillar: AnimGroup id PILLAR_ANIM_GROUP, pivot (4.2, 1.6, -1.0) (its own
        // authored Transform — see spawn_tile_cluster). The rest-pose Node's own
        // local frame is *pivot-relative* (see assemble_rest_pose_subtree — every
        // leaf isometry has `pivot` subtracted out), and the pillar carries no
        // extra rotation, so its own cylinder surface sits at local
        // (radius, 0, 0) = (0.6, 0, 0) relative to the pivot.
        let pillar = scene
            .anim_groups
            .iter()
            .find(|g| g.id == 0)
            .expect("pillar's AnimGroupScene should be present");
        assert_eq!(pillar.pivot, Vec3::new(4.2, 1.6, -1.0));

        let p = Vec3::new(0.6, 0.0, 0.0);
        let d = pillar.node.distance(p);
        assert!(d.abs() < 1e-3, "pillar rest-pose point not on surface: {d}");
    }

    #[test]
    fn tile_cluster_ground_does_not_reach_the_cell_boundary() {
        let (mut world, root) = test_app_with(spawn_tile_cluster);
        let cluster = world
            .run_system_once_with(
                move |shapes: Query<ShapeQueryData>,
                      children_of: Query<&Children>| {
                    assembly::assemble_scene(root, &shapes, &children_of)
                },
                (),
            )
            .unwrap()
            .unwrap();

        let edge = Vec3::new(TILE_PERIOD / 2.0, 0.0, 0.0);
        let d = cluster.distance(edge);
        assert!(
            d > 0.5,
            "tile cluster ground reaches too close to the cell boundary: distance={d}"
        );
    }
}
