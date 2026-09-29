---
title: Architecture
description: Archived top-level design of migera's former SDF-to-splat renderer: SDF as world source of truth, splats as the baked render target, and the Bevy RenderAsset/PhaseItem/Core3d pass scaffolding that made them native. Read before reconsidering that architecture or adding a custom Core3d geometry pass.
type: index
status: current
tags:
  - 3dgs
  - bevy
  - render-pipeline
  - integration
updated: 2026-09-28
---

# Architecture

The division of labour between SDF (world description) and 3DGS (rendered output), and
how a baked splat cloud became part of Bevy's `Core3d` schedule rather than a side
pipeline. Both notes are archived: the splat renderer was removed in commits 22d3b91
(2026-08-16) and 684490c (2026-09-06). The Bevy pass mechanics remain a useful reference
for any custom geometry pass.

## Archived / superseded

| Note | What it establishes | Read when |
|---|---|---|
| [SDF as world description, 3DGS as world rendering](./world-representation.md) | The SDF-authors/splats-render split modelled on Dreams, and the case (later reversed) against direct raymarching | Reopening SDF-to-splat vs. direct raymarch |
| [Native integration with Bevy's render pipeline](./bevy-pipeline-integration.md) | `SplatAsset` RenderAsset, opaque `Splat3d` binned phase, reused `Transparent3d`, pass ordering, shared `ViewTarget` | Adding any custom geometry pass to `Core3dSystems::MainPass` |
