---
title: Entity sync and the four extraction patterns
description: Bevy 0.19.1 keeps stable render-world mirrors of entities via RenderEntity/MainEntity and offers four ways to copy data across - ExtractComponentPlugin, ExtractInstancesPlugin, ExtractResourcePlugin, manual Extract<> systems. Read before adding render support for a component or when extracted data never shows up.
type: reference
status: current
tags:
  - bevy
  - render-pipeline
  - ecs
  - integration
  - debugging
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_render-0.19.1/src/sync_world.rs, extract_component.rs, extract_instances.rs, extract_resource.rs
aliases:
  - RenderEntity
  - MainEntity
  - SyncToRenderWorld
  - TemporaryRenderEntity
  - ExtractComponentPlugin
---

# Entity sync and the four extraction patterns

## Why this exists

Every frame, the render world must reflect the current state of the main world. Two
sub-problems need solving:

1. **Entity identity** — a game object's render-world "mirror" entity must be *stable*
   across frames (so GPU handles, cached bind groups, and batching bookkeeping tied to it
   aren't rebuilt every frame), not recreated wholesale each frame.
2. **Data copying** — main-world component/resource data needs a defined, ergonomic path
   into the render world, with sensible defaults for cleanup when data is removed.

## Entity identity: `sync_world.rs`

Two components form a persistent link between a main-world entity and its render-world
mirror:

- **`RenderEntity(Entity)`** lives on the *main-world* entity, pointing at its render-world
  mirror.
- **`MainEntity(Entity)`** lives on the *render-world* entity, pointing back at the source.
  Both implement `QueryData` in a way that lets you write `Query<(MainEntity, ...)>`
  without a `&` and get a bare `Entity` back — a deliberate ergonomic shortcut.

A main-world entity opts in by having the marker component `SyncToRenderWorld` (usually
added indirectly — see below, not by hand). `SyncWorldPlugin` registers ECS observers on
`Add<SyncToRenderWorld>`/`Remove<SyncToRenderWorld>` that queue `EntityRecord`s
(`Added`/`Removed`/`ComponentRemoved`) into a `PendingSyncEntity` resource. Once per frame,
`entity_sync_system` (run as the very first step of extraction, before `ExtractSchedule`
itself) drains that queue:

- `Added(e)` → spawns a **new** render-world entity carrying only `MainEntity(e)`, writes
  the resulting id back into the main entity's `RenderEntity`.
- `Removed(render_entity)` → despawns that render-world entity outright.
- `ComponentRemoved(main_entity, removal_fn)` → calls a stored function pointer that
  removes one specific bundle from the mirrored render entity, without despawning it.

Because spawn/despawn only happens on actual add/remove of `SyncToRenderWorld`, **the
render-world entity ID for a given game object is stable across many frames** — everything
else about it is refreshed *in place* by extraction systems each frame.

`TemporaryRenderEntity` is a separate marker for render-world-only entities with no
main-world counterpart (helper entities spawned during extraction, e.g. shadow-view
entities). `despawn_temporary_render_entities` (`RenderSystems::PostCleanup`) despawns all
of them every frame, sorted by entity index for determinism, so they never accumulate.

`SyncComponent<F = ()>: Component` (`sync_component.rs`) is the trait that connects a
specific main-world component type to this machinery: `Target: Bundle` names what to
remove from the render mirror when the component is removed. `SyncComponentPlugin<C, F>`
auto-requires `SyncToRenderWorld` on any entity that gets a `C` component (via
`register_required_components`) and installs the `on_remove` hook that queues the
`ComponentRemoved` record — this is why users almost never add `SyncToRenderWorld` by
hand.

## The four extraction patterns

### 1. `ExtractComponentPlugin<C>` — per-entity component copying

`ExtractComponent<F = ()>: SyncComponent<F>` declares `QueryData`, `QueryFilter`, `Out:
Bundle`, and `fn extract_component(item) -> Option<Self::Out>`. `ExtractComponentPlugin`
adds a system to `ExtractSchedule` that queries the main world for `(RenderEntity,
C::QueryData)`, transforms each item, and batches all inserts via
`commands.try_insert_batch(...)` on the render world. Returning `None` removes
`SyncComponent::Target` from the render entity instead of inserting.

```rust
#[derive(Component, Clone, ExtractComponent)]
struct Glow(f32);
app.add_plugins(ExtractComponentPlugin::<Glow>::default());
```

The derive macro generates the trivial identity case (`QueryData = &'static Self`, `Out =
Self`, clone-and-return). `#[extract_component_filter(F)]` and
`#[extract_component_sync_target(T)]` customize the filter/target.

**Use when**: the mapping is "one main-world component → one render-world component/
bundle," and you want automatic add/remove-on-despawn wiring for free. There's also an
`extract_visible()` constructor variant that additionally requires `&ViewVisibility` and
skips culled entities — useful for expensive-to-extract data.

### 2. `ExtractInstancesPlugin<EI>` — high-frequency lookup-keyed data

`ExtractInstance` mirrors `ExtractComponent`'s shape but has **no** tie to
`SyncComponent`/render-world entities at all. Extracted values go into a single resource,
`ExtractedInstances<EI>(MainEntityHashMap<EI>)`, cleared and rebuilt every frame, keyed
directly by the *main-world* `Entity` (wrapped as `MainEntity`) — no dependency on
`SyncToRenderWorld` or the entity-sync machinery.

**Use when**: you need "look up extracted data for main entity X" rather than an ECS-query
join, especially at scale (many thousands of entities) where avoiding per-entity
archetype/component insertion overhead matters more than composability with other
render-world components.

### 3. `ExtractResourcePlugin<R>` — whole-resource copying

`ExtractResource<F = ()>: Resource` declares `type Source: Resource` and `fn
extract_resource(source: &Self::Source) -> Self`. The system it registers uses
`main_resource.is_changed()` to skip re-copying (and thus skip whatever cost `R::
extract_resource` has) when the source hasn't mutated since last frame.

**Use when**: a global/singleton piece of render-relevant configuration lives as a main-
world `Resource` and needs a render-world copy — simpler than faking a resource through
per-entity extraction.

### 4. Manual extraction with `Extract<P>`

For cases the three plugins above don't fit — aggregating across many entities into one
resource, needing custom per-frame bookkeeping, or wanting to skip ECS insertion overhead
differently than `ExtractInstance` does — write a system directly on `ExtractSchedule`
using the `Extract<P>` system param, which resolves any read-only `SystemParam` (typically
a `Query`) against the main world instead of the render world:

```rust
fn extract_clouds(mut commands: Commands, clouds: Extract<Query<RenderEntity, With<Cloud>>>) {
    for cloud in &clouds {
        commands.entity(cloud).insert(Cloud);
    }
}
```

Note the query fetches `RenderEntity` (not `Entity`) — that's the render-world id to
target with `commands`, since `commands` here operates on the render world. `Extract` is
constrained to `ReadOnlySystemParam`: writing main-world data from `ExtractSchedule` is
intentionally impossible through this API.

## When to dive in

- Adding render support for a new component type → start with
  `ExtractComponentPlugin`/derive; only reach for `ExtractInstancesPlugin` or manual
  `Extract<...>` systems if you've measured a real cost or need cross-entity aggregation.
- Debugging "my extracted data doesn't show up" → check whether the source component even
  has `SyncToRenderWorld` (usually implicit via `ExtractComponentPlugin`/`SyncComponent`);
  for `ExtractInstance`/manual approaches there's no such requirement.
- Writing a plugin that needs render-world-only scratch entities → use
  `TemporaryRenderEntity`, don't hand-roll your own cleanup.

## Related
- [The RenderApp split and the Extract step](./render-app-and-extraction.md) — prerequisite: the two-world model and `ExtractSchedule` these patterns plug into.
- [RenderAsset lifecycle](../resources-and-assets/render-assets.md) — contrast: the separate extract/prepare path for assets (`Mesh`, `Image`) rather than components.
- [Gizmo rendering](../2d-and-ui/gizmo-rendering.md) — example: `TemporaryRenderEntity` used for render-world-only entities with no main-world source.
- [Bevy-native integration](../../hybrid-architecture/bevy-native-integration.md) — applies: migera's rule to extract from real Bevy components (camera, lights) in `src/hybrid`.
