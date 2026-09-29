---
title: "bevy_shader: WGSL, preprocessing, and the shader cache"
description: Bevy 0.19.1's bevy_shader crate provides the Shader asset (WGSL, WESL, GLSL, SPIR-V), the naga_oil preprocessor (#import, #ifdef, ShaderDefVal), load_shader_library!, module-path import resolution, and ShaderCache with dependent invalidation. Read when a shader import fails, hot reload misses, or you add shader defs.
type: reference
status: current
tags:
  - bevy
  - wgsl
  - render-pipeline
  - debugging
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_shader-0.19.1/src
aliases:
  - naga_oil
  - shader imports
  - define_import_path
  - ShaderDefVal
  - shader hot reload
---

# bevy_shader: WGSL, preprocessing, and the shader cache

## Provenance

`bevy_shader` is a standalone crate ("Provides `Shader` assets for Bevy") split out of
`bevy_render`. Its migration guide entries confirm active, recent cleanup post-split.

## `Shader` asset and source formats

`Shader` (`shader.rs`) wraps a `Source` enum: `Wgsl(Cow<str>)`, `Wesl(Cow<str>)` (feature
`shader_format_wesl`), `Glsl(Cow<str>, naga::ShaderStage)` (feature `shader_format_glsl`),
`SpirV(Cow<[u8]>)` (feature `shader_format_spirv`, largely stubbed — several paths
`panic!("spirv not yet implemented")`). Construction helpers: `Shader::from_wgsl`,
`from_glsl`, `from_spirv`, `from_wesl`. `ShaderLoader` (an `AssetLoader`) dispatches by
file extension (`.wgsl`, `.vert`/`.frag`/`.comp`, `.spv`, `.wesl`) and accepts
`ShaderSettings { shader_defs: Vec<ShaderDefVal> }` so `#define`-style values can be
supplied at load time via `.meta` files or load-context settings.

## Preprocessor: `#import`, `#ifdef`, `ShaderDefVal`

Preprocessing (conditional compilation, imports, composition) is delegated to
**`naga_oil::compose::Composer`** — `bevy_shader` orchestrates naga_oil rather than
reimplementing a preprocessor. `Shader::preprocess` calls naga_oil's
`get_preprocessor_data` at load time purely to extract the shader's own
`#define_import_path` and its `#import` list, for dependency tracking.

`ShaderDefVal` is the value type for `#ifdef`/`#if` conditions and `#define`-style
constant substitution:

```rust
pub enum ShaderDefVal { Bool(String, bool), Int(String, i32), UInt(String, u32) }
```

A bare `&str` converts to `ShaderDefVal::Bool(name, true)` — the common case of a feature-
flag toggle. These are supplied per-specialization (e.g. `"VERTEX_TANGENTS"`,
`"STANDARD_MATERIAL_NORMAL_MAP"`) and become part of the shader-module cache key, so
different combinations compile to genuinely distinct GPU modules.

## `load_shader_library!`

```rust
macro_rules! load_shader_library {
    ($asset_server_provider: expr, $path: literal $(, $settings: expr)?) => { ... }
}
```

Expands to `embedded_asset!` + `load_embedded_asset!`, then `mem::forget`s the resulting
handle so the shader loads once and stays alive without the caller holding a handle. The
doc comment explains why: "This works around a limitation of the shader loader not
properly loading dependencies of shaders" — transitively imported `.wgsl` library files
(like `bevy_pbr::mesh_view_bindings`) need pre-registering as embedded assets so `#import`
resolution can find them, since the loader alone won't discover import-only dependencies.
`bevy_pbr`'s plugin `build()` calls this repeatedly for its shader library files. Anyone
writing a shared shader-library crate needs this same pattern.

## Import resolution and module paths

Each `.wgsl` file declares its logical import name via `#define_import_path
bevy_pbr::mesh_view_bindings` at the top; other shaders `#import
bevy_pbr::mesh_view_bindings::view` (or `as types`). This is a *logical namespace*,
unrelated to filesystem/crate paths — it's how `bevy_pbr::mesh_view_bindings` resolves
regardless of which crate physically contains the file, letting downstream crates or user
code `#import` bevy_pbr internals by name as long as that shader was registered
(typically via `load_shader_library!`) somewhere in the app.

## `ShaderCache`

```rust
pub struct ShaderCache<ShaderModule, RenderDevice> { device: RenderDevice, ... }
```

Both type parameters are generic, **not** concrete wgpu types — this exists "to avoid a
cyclic dependency with `bevy_render`, while also permitting alternative rendering
implementations." A migration-guide-documented change: `ShaderCache::new` now accepts a
`RenderDevice`, and `ShaderCache::get` does not — reflecting that "a `ShaderCache` must
only be used with one `RenderDevice` for it to be valid." The cache now owns the device
and passes it into `load_module` internally at `get()` time, preventing a cache being
queried against a mismatched device.

Internally, `get(pipeline, id, shader_defs)` resolves the full import graph (recursively
adding composable modules to the shared `naga_oil::compose::Composer`), merges the
shader's own baked-in `shader_defs` with caller-supplied ones, calls
`composer.make_naga_module(...)`, then invokes the renderer-supplied `load_module` to turn
a `naga::Module` into an actual GPU shader module — caching by `(AssetId<Shader>,
Box<[ShaderDefVal]>)`. `set_shader` handles hot-reload: replacing a shader clears its
processed cache and recursively invalidates all `dependents`, returning the
`CachedPipelineId`s that must be re-specialized (see
[pipeline-cache-and-specialization](../resources-and-assets/pipeline-cache-and-specialization.md)).

## When to dive in

- Writing any custom shader that needs to share code with Bevy's built-in shaders (e.g.
  reusing `mesh_view_bindings`) → use `#import`/`#define_import_path` and
  `load_shader_library!`.
- Debugging "shader import not found"/hot-reload not picking up changes → check that the
  library shader was registered via `load_shader_library!` (not just present on disk), and
  that `ShaderCache::set_shader`'s dependent-invalidation is actually being triggered.
- Adding conditional shader logic (feature flags, optional material features) → use
  `ShaderDefVal` and `#ifdef` blocks, matching them in your specialization key so distinct
  combinations get distinct compiled pipelines.

## Related
- [PipelineCache and pipeline specialization](../resources-and-assets/pipeline-cache-and-specialization.md) — deeper: how shader defs become distinct cached pipelines.
- [bevy_mesh: the mesh data model](./mesh-data-model.md) — contrast: the vertex-attribute side a shader must match.
- [Compute shaders in Bevy 0.19](../../compute-shaders/bevy-integration.md) — applies: loading and binding migera's WGSL compute shaders.
