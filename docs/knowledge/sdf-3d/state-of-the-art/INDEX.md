---
title: State of the Art
description: What around SDFs is proven-and-shipping vs. still active research as of 2025-2026 — neural/learned SDFs, Unreal Lumen's production hybrid, the modeling-tool landscape, and a dated research snapshot. Read when calibrating how settled a technique is before betting a feature on it.
type: index
status: current
tags:
  - sdf
  - state-of-the-art
  - prior-art
updated: 2026-09-28
---

# State of the Art

The classical techniques (primitives, CSG, sphere tracing) have been stable for about
a decade, but three areas are moving fast as of 2025-2026: neural/learned
representations, production hybrid architectures at AAA scale, and mainstream
content-creation tooling. These notes are dated snapshots (written 2026-08-15);
re-check anything load-bearing.

## Key facts

- The Eikonal constraint is only a soft training loss in neural SDFs, so their gradients off-surface are unreliable — see [neural-and-learned-sdfs](./neural-and-learned-sdfs.md).
- Lumen uses SDFs for indirect-light rays only; primary visibility stays rasterized/HW-RT — see [unreal-lumen-distance-fields](./unreal-lumen-distance-fields.md).
- SDF/Gaussian-splatting hybrids are active research, not settled practice — see [recent-research-directions](./recent-research-directions.md).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [Neural and learned SDFs](./neural-and-learned-sdfs.md) | DeepSDF, SIREN, Eikonal loss, NeuS, unsigned/orthogonal variants, real-time status. | Considering neural SDFs, or a learned SDF has bad normals. |
| [Unreal Engine's Mesh Distance Fields and Lumen](./unreal-lumen-distance-fields.md) | Per-mesh baked fields, clipmap Global Distance Field, Detail/Global software tracing. | Designing a hybrid SDF architecture or SDF-based GI. |
| [SDF-based content-creation tools](./sdf-modeling-tools.md) | Substance 3D Modeler, Uniform, Blender SDF nodes; the shared node-editor + preview + OpenVDB pattern. | Choosing or designing an SDF authoring tool. |
| [Recent research directions (2024-2026)](./recent-research-directions.md) | iSDF, fast GWN, differentiable isosurfaces, SDF/3DGS hybrids, hardware trends, non-SDF GI. | Judging how mature a technique is. |

## See also

- [3DGS state of the art](../../3dgs/INDEX.md) — the splatting side of SDF/3DGS hybrids.
- [SDF + 3DGS + Bevy integration](../../sdf-3dgs-bevy-integration/INDEX.md) — migera's archived synthesis of the two (splat renderer removed by 2026-09-06).
