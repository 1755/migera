//! Gets scene/light data from the main world into the render world for the raymarcher
//! to upload to the GPU. Split into two halves:
//!
//! - `rebuild_raymarch_static_scene` (main-world, `Update`): reuses
//!   `sdf::world::assemble_infinite_scene` to assemble the tile cluster's `Node` tree,
//!   then `raymarch::flatten::flatten_scene`s it into GPU-uploadable records — but
//!   only once, not every frame (see that function's doc comment for why).
//! - `extract_raymarch_*` (`ExtractSchedule`): cheap `Vec`/`Resource` copies from main
//!   world into render world, the standard manual-extraction convention.
//!
//! `Prepare`-schedule systems (`raymarch::pipeline`) then own turning these plain
//! `Vec`s into actual GPU `RawBufferVec`s + bind groups.

use bevy::asset::Assets;
use bevy::math::{Quat, Vec3};
use bevy::prelude::*;
use bevy::render::Extract;
use bevy::shader::Shader;

use crate::sdf::assembly::ShapeQueryData;
use crate::sdf::components::{AnimGroup, ProceduralPattern};
use crate::sdf::world::{TileClusterRoot, assemble_infinite_scene};

use super::flatten::{FlattenedScene, PrimitiveRecordCpu, flatten_scene};

/// The logical import path `raymarch.wgsl` itself `#import`s to reach the generated
/// pattern dispatcher (see `dispatcher_shader_source`) — fixed and known at compile
/// time, unlike the *pattern* import paths themselves (which come from user-authored
/// `ProceduralPattern` components and vary per scene). `raymarch.wgsl` never needs to
/// change when patterns are added/removed from a scene; only the shader asset behind
/// this one fixed path does.
pub const DISPATCHER_IMPORT_PATH: &str = "migera::pattern_dispatch";

/// Generates the WGSL source for the pattern dispatcher: one `#import` per distinct
/// pattern in `registry` (aliased `pattern_0`, `pattern_1`, ...) and a `switch`
/// calling each one's `evaluate_pattern` — the "real codegen" piece this project's
/// extensible-materials design needs, since WGSL has no function pointers, so
/// dynamically dispatching to a scene-chosen, dynamically-loaded pattern function
/// still needs a concrete `switch` arm generated per active pattern (see `sdf::
/// components::ProceduralPattern`'s doc comment for the full design rationale and
/// `assets/shaders/patterns/checkerboard.wgsl` for the reference pattern-authoring
/// contract this dispatches to). Regenerated (and re-registered via `Assets<Shader>`)
/// every time `PatternRegistry`'s entry set changes — seeing `switch`'s `default` arm
/// hit at runtime means a record's `pattern_id` refers to a pattern this dispatcher
/// doesn't know about, which should never happen if `PatternRegistry`/`pattern_id`
/// resolution (see `resolve_pattern_ids`) stay in sync with this generation.
fn dispatcher_shader_source(registry: &PatternRegistry) -> String {
    let mut imports = String::new();
    let mut arms = String::new();
    for (i, entry) in registry.entries.iter().enumerate() {
        imports.push_str(&format!("#import {} as pattern_{i}\n", entry.import_path));
        arms.push_str(&format!(
            "        case {}u: {{ return pattern_{i}::evaluate_pattern(p, mat_a, mat_b, params); }}\n",
            i + 1
        ));
    }

    format!(
        "#define_import_path {DISPATCHER_IMPORT_PATH}\n\
         #import migera::material::Material\n\
         {imports}\n\
         fn dispatch_pattern(pattern_id: u32, p: vec3<f32>, mat_a: Material, mat_b: Material, params: vec4<f32>) -> Material {{\n\
         \x20   switch (pattern_id) {{\n\
         {arms}\
         \x20       default: {{ return mat_a; }}\n\
         \x20   }}\n\
         }}\n"
    )
}

/// The dispatcher shader's own stable `Handle<Shader>` (reserved once, see
/// `rebuild_raymarch_static_scene`'s first-run branch) — kept at a fixed `AssetId` so
/// every regeneration replaces the *same* asset (via `Assets<Shader>::insert`, an
/// update rather than a fresh `add`), which is what lets Bevy's `ShaderCache`
/// transitively invalidate/recompile `raymarch.wgsl`'s own compiled pipeline whenever
/// the dispatcher's source changes (see `bevy_shader::ShaderCache::set_shader`'s
/// dependent-invalidation, keyed by `AssetId` — a fresh `AssetId` every regeneration
/// would leave `raymarch.wgsl`'s `#import` pointed at a stale, orphaned module
/// instead).
#[derive(Resource)]
pub struct DispatcherShaderHandle(pub Handle<Shader>);

/// Main-world cache of the tile cluster's flattened scene — built once (see
/// `rebuild_raymarch_static_scene`) and read every frame by
/// `extract_raymarch_static_scene`. `built` gates re-flattening: this demo's ECS
/// Shape/Blend/SdfSceneRoot hierarchy never changes shape after `spawn_tile_cluster`
/// runs (only `AnimGroup` entities' `Transform.rotation` changes, and those are
/// already handled separately/every-frame by `extract_raymarch_anim_isometries`), so
/// re-flattening every frame would repeat identical work for no benefit — static
/// geometry flattens once, live rotation reapplies every frame instead.
#[derive(Resource, Default)]
pub struct CachedRaymarchScene {
    pub flattened: Option<FlattenedScene>,
}

/// One entry in `PatternRegistry` — a distinct `ProceduralPattern::import_path` seen
/// anywhere in the scene, plus the `Handle<Shader>` that path's shader was loaded
/// through. `dispatcher_shader_source` reads `import_path` to generate the dispatcher
/// WGSL module's `#import`s and `switch (pattern_id)` arms (see `sdf::components::
/// ProceduralPattern`'s doc comment for the pattern-authoring contract this
/// dispatches to). `shader` itself is never read directly — it exists purely to keep
/// the pattern shader asset's `Handle` (and thus the loaded asset) alive for as long
/// as the registry holds this entry, the same "held only for its strong-reference
/// side effect" role `raymarch::pipeline::RaymarchPipeline::material_shader` plays
/// for `material.wgsl`.
#[derive(Clone)]
pub struct PatternRegistryEntry {
    pub import_path: String,
    #[allow(dead_code)]
    pub shader: Handle<Shader>,
}

/// The full set of distinct patterns actually referenced by `ProceduralPattern`
/// components in the current scene, built once alongside `CachedRaymarchScene` (see
/// `rebuild_raymarch_static_scene`) — `entries[i]`'s registry index is `i + 1`
/// (`pattern_id == 0` is reserved for "no pattern, shade directly from base_color/
/// metallic/roughness", matching `PrimitiveRecordCpu::pattern_id`'s doc comment).
/// Deduplicated by `import_path` (not by `Handle<Shader>`/entity) so two entities
/// pointing at the same pattern shader share one dispatcher `switch` arm and one
/// `#import`, rather than the generated dispatcher growing one arm per *entity*
/// instead of per distinct *pattern*.
#[derive(Resource, Default, Clone)]
pub struct PatternRegistry {
    pub entries: Vec<PatternRegistryEntry>,
}

impl PatternRegistry {
    /// 1-based index for `import_path` in this registry (`None` if not present) —
    /// `PrimitiveRecordCpu::pattern_id` uses `0` for "no pattern," so a real entry's
    /// GPU-facing id is always its position here plus one.
    pub fn pattern_id(&self, import_path: &str) -> Option<u32> {
        self.entries
            .iter()
            .position(|e| e.import_path == import_path)
            .map(|i| i as u32 + 1)
    }
}

/// Live per-`AnimGroup` isometry (pivot stays fixed, rotation updates every frame) —
/// render-world mirror of what the shader's `AnimIsometry` storage-buffer entry needs.
/// `group_id` matches `sdf::components::AnimGroup`'s id (0 = pillar, per
/// `sdf::world`'s `PILLAR_ANIM_GROUP`).
#[derive(Clone, Copy)]
pub struct AnimIsometryCpu {
    pub group_id: u32,
    pub pivot: Vec3,
    pub rotation: Quat,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum LightKindCpu {
    Directional,
    Point,
    Spot,
}

/// One light's render-time data, flattened from whichever of Bevy's
/// `DirectionalLight`/`PointLight`/`SpotLight` it came from — see
/// `extract_raymarch_lights`. `direction_or_position`/`spot_direction` match
/// `raymarch.wgsl`'s `Light` struct field-for-field (see that shader's doc comment for
/// the per-kind meaning).
#[derive(Clone, Copy)]
pub struct LightCpu {
    pub kind: LightKindCpu,
    pub color: Vec3,
    pub direction_or_position: Vec3,
    pub intensity: f32,
    pub spot_direction: Vec3,
    pub range: f32,
    pub inner_angle: f32,
    pub outer_angle: f32,
}

/// Render-world copy of `CachedRaymarchScene.flattened`'s static-scene half — the
/// large (~9-primitive) buffer that only needs uploading once, not every frame (see
/// `CachedRaymarchScene`'s doc comment).
#[derive(Resource, Default)]
pub struct RenderRaymarchStaticScene {
    pub tile_period: f32,
    pub records: Vec<PrimitiveRecordCpu>,
    /// Each `AnimGroup`'s flattened rest-pose records, concatenated (matches
    /// `FlattenedScene::anim_group_records`) — static too (only the entries in
    /// `RenderRaymarchAnimIsometries` change per frame), so this rides along with the
    /// static-scene one-time upload rather than being re-derived every frame.
    pub anim_group_records: Vec<PrimitiveRecordCpu>,
    pub anim_group_ranges: Vec<(u32, u32, u32)>, // (group_id, start, count) into anim_group_records
}

#[derive(Resource, Default)]
pub struct RenderRaymarchAnimIsometries(pub Vec<AnimIsometryCpu>);

#[derive(Resource, Default)]
pub struct RenderRaymarchLights(pub Vec<LightCpu>);

/// Bitmask of debug flags fed into the shader's `scene.debug_flags` uniform every
/// frame. Main-world resource, toggled by keyboard in the example; extracted into the
/// render world each frame (trivial copy of one `u32`).
#[derive(Resource, Clone, Copy, Default)]
pub struct RaymarchDebugFlags(pub u32);

/// `ExtractSchedule`, every frame: copies the current debug flags into the render
/// world.
pub fn extract_raymarch_debug_flags(
    mut commands: Commands,
    flags: Extract<Res<RaymarchDebugFlags>>,
) {
    commands.insert_resource(RaymarchDebugFlags(flags.0));
}

/// Main-world, `Update`: assembles + flattens the tile cluster's `Node` tree into
/// `CachedRaymarchScene`, once, and builds `PatternRegistry` from every distinct
/// `ProceduralPattern::import_path` referenced anywhere in the scene (reading
/// `shapes` directly for this rather than trying to recover it from the flattened
/// output, which only carries bare import-path strings — see `PrimitiveRecordCpu::
/// pattern_id`'s doc comment for why: GPU records can't hold a `Handle<Shader>`, only
/// the ECS `ProceduralPattern` component the ECS query below still has access to).
/// Cheap to check every frame (`cached.flattened.is_some()`), negligible cost to skip
/// — the actual assemble+flatten work only ever runs the one time `TileClusterRoot`
/// first exists with its full hierarchy spawned.
#[allow(clippy::too_many_arguments)]
pub fn rebuild_raymarch_static_scene(
    mut commands: Commands,
    tile_cluster_root: Option<Res<TileClusterRoot>>,
    shapes: Query<ShapeQueryData>,
    children_of: Query<&Children>,
    anim_groups_query: Query<&AnimGroup>,
    transforms: Query<&GlobalTransform>,
    patterns: Query<&ProceduralPattern>,
    mut cached: ResMut<CachedRaymarchScene>,
    mut registry: ResMut<PatternRegistry>,
    dispatcher_handle: Option<Res<DispatcherShaderHandle>>,
    mut shaders: ResMut<Assets<Shader>>,
) {
    if cached.flattened.is_some() {
        return;
    }
    let Some(tile_cluster_root) = tile_cluster_root else {
        return; // setup() hasn't spawned the tile cluster yet — try again next frame.
    };

    let mut registry_changed = false;
    for pattern in &patterns {
        if registry.pattern_id(&pattern.import_path).is_none() {
            registry.entries.push(PatternRegistryEntry {
                import_path: pattern.import_path.clone(),
                shader: pattern.shader.clone(),
            });
            registry_changed = true;
        }
    }

    // Generate (first run) or regenerate (registry grew) the dispatcher shader and
    // register/update it at a stable AssetId — see `DispatcherShaderHandle`'s doc
    // comment for why a stable id (via `Assets::insert` on an already-reserved
    // handle, not a fresh `Assets::add` every time) is what lets `raymarch.wgsl`'s
    // own compiled pipeline get transitively invalidated/recompiled through Bevy's
    // shader-dependency tracking whenever the active pattern set changes.
    let dispatcher_handle = match dispatcher_handle {
        Some(existing) => existing.0.clone(),
        None => {
            let handle = shaders.reserve_handle();
            commands.insert_resource(DispatcherShaderHandle(handle.clone()));
            handle
        }
    };
    if registry_changed || !shaders.contains(&dispatcher_handle) {
        let source = dispatcher_shader_source(&registry);
        shaders
            .insert(&dispatcher_handle, Shader::from_wgsl(source, "generated://pattern_dispatch.wgsl"))
            .expect("dispatcher_handle is a freshly reserved index-based handle, whose insert cannot fail");
    }

    let scene = assemble_infinite_scene(
        tile_cluster_root.0,
        &shapes,
        &children_of,
        &anim_groups_query,
        &transforms,
    );
    let mut flattened = flatten_scene(&scene);
    resolve_pattern_ids(
        &mut flattened.static_records,
        &flattened.static_pattern_import_paths,
        &registry,
    );
    resolve_pattern_ids(
        &mut flattened.anim_group_records,
        &flattened.anim_group_pattern_import_paths,
        &registry,
    );
    cached.flattened = Some(flattened);
}

/// Writes each record's real `pattern_id` (see `PrimitiveRecordCpu::pattern_id`'s doc
/// comment) from the parallel `import_paths` list, now that `registry` — built just
/// before this is called — has an entry for every pattern referenced anywhere in the
/// scene. `import_paths[i]` corresponds to `records[i]`; a `None` entry (no
/// `ProceduralPattern` on that leaf's source entity) leaves `pattern_id` at its
/// already-`0` default from `PrimitiveRecordCpu::leaf`/`op`.
fn resolve_pattern_ids(
    records: &mut [PrimitiveRecordCpu],
    import_paths: &[Option<String>],
    registry: &PatternRegistry,
) {
    for (record, import_path) in records.iter_mut().zip(import_paths) {
        if let Some(import_path) = import_path {
            record.pattern_id = registry
                .pattern_id(import_path)
                .expect("every import_path in FlattenedScene should have been registered into PatternRegistry above");
        }
    }
}

/// `ExtractSchedule`: copies the static scene into the render world — only actually
/// needs to happen once (the source `CachedRaymarchScene` itself only changes once),
/// but re-checking a cheap `Option::is_none()` on the render-world resource every frame
/// is simpler than threading a separate one-shot flag through `Extract`, and costs
/// nothing once populated.
pub fn extract_raymarch_static_scene(
    mut commands: Commands,
    cached: Extract<Res<CachedRaymarchScene>>,
    existing: Option<Res<RenderRaymarchStaticScene>>,
) {
    if existing.is_some() {
        return;
    }
    let Some(flattened) = &cached.flattened else {
        return;
    };
    let anim_group_ranges = flattened
        .anim_groups
        .iter()
        .map(|g| (g.id, g.start, g.count))
        .collect();
    commands.insert_resource(RenderRaymarchStaticScene {
        tile_period: flattened.tile_period,
        records: flattened.static_records.clone(),
        anim_group_records: flattened.anim_group_records.clone(),
        anim_group_ranges,
    });
}

/// `ExtractSchedule`, every frame: each live `AnimGroup` entity's current pivot +
/// rotation.
pub fn extract_raymarch_anim_isometries(
    mut commands: Commands,
    live_groups: Extract<Query<(&AnimGroup, &Transform)>>,
) {
    let isometries: Vec<AnimIsometryCpu> = live_groups
        .iter()
        .map(|(group, transform)| AnimIsometryCpu {
            group_id: group.0,
            pivot: transform.translation,
            rotation: transform.rotation,
        })
        .collect();
    commands.insert_resource(RenderRaymarchAnimIsometries(isometries));
}

/// `ExtractSchedule`, every frame: flattens every `DirectionalLight`/`PointLight`/
/// `SpotLight` in the scene into one `Vec<LightCpu>` — direction/position comes from
/// `GlobalTransform` (not the light components themselves), per Bevy's convention:
/// directional light direction = transform's forward, point/spot light position =
/// `transform.translation()`, spot direction = transform's forward.
pub fn extract_raymarch_lights(
    mut commands: Commands,
    dir_lights: Extract<Query<(&DirectionalLight, &GlobalTransform)>>,
    point_lights: Extract<Query<(&PointLight, &GlobalTransform)>>,
    spot_lights: Extract<Query<(&SpotLight, &GlobalTransform)>>,
) {
    let mut lights = Vec::new();

    for (light, transform) in &dir_lights {
        lights.push(LightCpu {
            kind: LightKindCpu::Directional,
            color: light.color.to_linear().to_vec3(),
            direction_or_position: transform.forward().as_vec3(),
            intensity: light.illuminance,
            spot_direction: Vec3::ZERO,
            range: 0.0,
            inner_angle: 0.0,
            outer_angle: 0.0,
        });
    }

    for (light, transform) in &point_lights {
        lights.push(LightCpu {
            kind: LightKindCpu::Point,
            color: light.color.to_linear().to_vec3(),
            direction_or_position: transform.translation(),
            intensity: light.intensity,
            spot_direction: Vec3::ZERO,
            range: light.range,
            inner_angle: 0.0,
            outer_angle: 0.0,
        });
    }

    for (light, transform) in &spot_lights {
        lights.push(LightCpu {
            kind: LightKindCpu::Spot,
            color: light.color.to_linear().to_vec3(),
            direction_or_position: transform.translation(),
            intensity: light.intensity,
            spot_direction: transform.forward().as_vec3(),
            range: light.range,
            inner_angle: light.inner_angle,
            outer_angle: light.outer_angle,
        });
    }

    commands.insert_resource(RenderRaymarchLights(lights));
}

/// Render-world copy of `PatternRegistry` — `raymarch::pipeline` reads this every
/// frame to decide the current `RaymarchPipelineKey`'s active-pattern set (see that
/// type's doc comment) and to generate the dispatcher shader source. Extracted every
/// frame like `RenderRaymarchLights` (cheap: a handful of entries at most, and
/// `PatternRegistry` only actually grows the one time new patterns are first seen —
/// see `rebuild_raymarch_static_scene`) rather than gated behind an `is_some()` check
/// like `RenderRaymarchStaticScene`, since re-cloning a short `Vec` every frame is not
/// worth a separate one-shot-extraction code path.
pub fn extract_raymarch_pattern_registry(
    mut commands: Commands,
    registry: Extract<Res<PatternRegistry>>,
) {
    commands.insert_resource((*registry).clone());
}

/// Registers the main-world `Update` system that feeds the extraction systems above —
/// called from `RaymarchRenderPlugin::build`, which registers the `ExtractSchedule`
/// systems themselves alongside every other render-world system it owns.
pub fn build_main_world(app: &mut App) {
    app.init_resource::<CachedRaymarchScene>()
        .init_resource::<PatternRegistry>()
        .init_resource::<RaymarchDebugFlags>()
        .add_systems(Update, rebuild_raymarch_static_scene);
}
