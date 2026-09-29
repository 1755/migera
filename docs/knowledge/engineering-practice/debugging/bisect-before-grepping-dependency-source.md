---
title: Bisect your own code before grepping dependency source
description: When a bug could be in migera's code or a dependency's internals, bisect the app's own systems and plugins first; the search is bounded and the cause may sit in a third dependency. The invisible egui panel was Bevy's shadow-view camera, not bevy_egui. Read when a bug might live inside Bevy or a plugin.
type: lesson
status: current
tags:
  - debugging
  - bevy
  - ecs
updated: 2026-09-21
verified: 2026-09-28
code:
  - examples/character_gallery.rs
sources:
  - Claude memory bisect_before_grepping_dependency_source (2026-09-21)
aliases:
  - egui panel invisible
  - bevy_egui primary context
  - auto_create_primary_context
  - bisection
---

# Bisect your own code before grepping dependency source

When a bug could come from migera's own code or from a dependency (Bevy,
bevy_egui, ...), bisect the app's **own** systems and plugins first. Comment
out `Startup`/`Update` systems in halves, rebuild, re-check. Only then read
the dependency's source.

## What happened

On 2026-09-21 the egui Controls panel of `character_gallery` was invisible in
both interactive and `--shot` runs.
- **Dead end:** grepping `bevy_egui` and `bevy_ui_render` source for "what
  spawns a stray `Camera` entity" was long and found nothing. The spawn was not
  in either crate.
- **What worked:** about five cycles of bisecting the app's own `Startup`
  systems, checking `RUST_LOG=warn` output after each rebuild, found the
  culprit system directly.
- **Root cause:** `spawn_light` set `shadow_maps_enabled: true` on a
  `DirectionalLight`. Bevy's own shadow-mapping code then creates an internal
  shadow-view entity that carries a bare `Camera` component (it logs a "doesn't
  have a render graph configured" warning). `bevy_egui` 0.42's
  `EguiGlobalSettings::auto_create_primary_context` (on by default) attaches the
  primary egui context to the first entity that gains a `Camera`, without
  checking that it can render. The shadow view registered first, so the UI
  rendered every frame to a camera that never draws to screen. No panic, no
  egui warning.
- **Fix** (`examples/character_gallery.rs`, `spawn_camera`): set
  `auto_create_primary_context = false` and put `bevy_egui::PrimaryEguiContext`
  on the real camera explicitly. `gallery.rs` never hit this because its light
  does not request shadows.

## Why it matters

Bisecting your own N systems is a bounded search with a guaranteed answer.
Grepping a large, unfamiliar dependency is unbounded and may not contain the
cause at all: here the symptom surfaced in bevy_egui but the cause was in
Bevy's core rendering.

## How to apply

1. List your own systems/plugins that could be involved.
2. Disable half, rebuild, re-check. Repeat until one system is left.
3. Read dependency source only for the mechanism *after* you know which of your
   systems triggers it.

## Evidence

- `examples/character_gallery.rs`: `auto_create_primary_context = false` and
  `PrimaryEguiContext` on the real camera.

## Related
- [Verify, don't assert from memory](./verify-dont-assert-from-memory.md) — same-trap: the companion lesson from the same investigation.
- [The symptom is far from the cause in the rig chain](./symptom-is-far-from-cause-in-the-rig-chain.md) — same-trap: measure each link instead of guessing the likely culprit.
- [migera moves characters to Bevy's PBR pipeline](../../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md) — example: the milestone during which this bug was found and fixed.
