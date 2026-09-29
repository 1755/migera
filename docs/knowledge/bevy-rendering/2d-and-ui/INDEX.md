---
title: 2D and UI
description: Bevy 0.19.1 rendering outside the 3D PBR pipeline - Sprite and Mesh2d batching, immediate-mode gizmos (2D on top, 3D depth-tested), and UI from Node to pixels - three answers to how many small things become few draw calls. Read for 2D draw calls, debug drawing, or custom UI rendering.
type: index
status: current
tags:
  - bevy
  - render-pipeline
  - rasterization
updated: 2026-09-28
---

# 2D and UI

Everything that isn't part of the 3D PBR pipeline: 2D sprite/mesh rendering,
immediate-mode debug gizmos, and UI. All three use sorted phases (`Transparent2d`,
`Transparent3d`, `TransparentUi`) and the camera-driven `Core2d`/`Core3d` scheduling from
[Core Pipeline](../core-pipeline/INDEX.md), but each solves "how do many small things
become few draw calls" differently: CPU dynamic-buffer sprite batching, GPU-driven Mesh2d
batching, UI's adjacency-only atlas batching, and gizmos' single pre-built buffer.

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [2D rendering: Sprite, Mesh2d, and the two batching strategies](./sprite-and-mesh2d-rendering.md) | `bevy_sprite`/`bevy_sprite_render` split; CPU-batched `Sprite` vs GPU-driven `Mesh2d`/`Material2d`; transitional `SpriteMesh`; sprites never write depth. | 2D draw calls explode, 2D layering is wrong, or writing a 2D material. |
| [Gizmo rendering: immediate-mode debug drawing](./gizmo-rendering.md) | `GizmoAsset` as a `RenderAsset`; thick lines as vertex-pulled quads; 2D gizmos sort to `+INFINITY`, 3D gizmos are depth-tested (`depth_bias`). | A gizmo is hidden or mis-sized, or you are writing debug visualization. |
| [UI rendering: from Node to pixels](./ui-rendering.md) | SDF rounded rects/borders, `TransparentUi` with `stack_z_offsets`, adjacency-only batching, Parley/Swash text, shadows and gradients. | Building custom UI rendering, or UI layering/clipping bugs. |

## See also

- [Gizmos need --show-real-mesh off](../../character-animation/animation-core/gizmos-need-show-real-mesh-off.md) — applies: the migera verification trap that follows from 3D gizmo depth testing.
