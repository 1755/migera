---
title: Knowledge Index
description: Root map of migera's knowledge base — lists every domain with what it covers and when to open it. Start here before designing, debugging or changing any subsystem.
type: index
status: current
tags:
  - hybrid-renderer
  - character-animation
  - sdf
  - bevy
updated: 2026-09-28
---

# Knowledge Index

migera's long-term memory: decisions, measured results, bug lessons, and
distilled outside research. **How to read, write, tag, link and retire notes
is specified in [AGENTS.md](./AGENTS.md). Read it before adding or editing a
note.** Tag vocabulary: [TAGS.md](./TAGS.md).

**Navigate:**
1. Pick a domain below by its *Read when* column.
2. Open its `INDEX.md`, then the topic `INDEX.md`, then the note.
3. Stop as soon as the rows stop matching your task.

**Search instead:**
- `python3 tools/kb.py catalog` prints every note with its one-line description.
- `python3 tools/kb.py find <tag>...` lists the notes that carry those tags.
- `python3 tools/kb.py code <path>` lists the notes about a file. Run it
  before you change that file.

## Project knowledge: what migera built, decided, and learned

| Domain | What it covers | Read when |
|---|---|---|
| [Character animation](./character-animation/INDEX.md) | The rotation-space procedural animation stack in `src/character/anim`: rig and retargeting, IK and locomotion, springs, the active ragdoll. Also the lessons from its bugs and the prior art it drew on (Lugaru, botica). | Before touching `src/character`, poses, IK, gait or the ragdoll, or when a character looks wrong. |
| [Gameplay camera](./gameplay-camera/INDEX.md) | The third-person camera design for `src/camera` and its architecture decision (one rig, blended layers, post-blend collision). Also distilled research on shipped games (Gothic, Journey, Witcher 3, Skyrim, Souls), engine systems (Cinemachine, Unreal, Lyra, dolly), collision/occlusion and damping. | Before any camera follow, collision, lock-on, mode or input-feel work. |
| [Engineering practice](./engineering-practice/INDEX.md) | Cross-cutting lessons on testing, debugging and measurement that were paid for in real bugs. Examples: tests that could not fail, misleading measurements, stale live state. | Before writing a test that a bug fix relies on, before trusting a measurement or a live-state readout, and whenever a diagnosis feels stuck. |
| [Hybrid renderer architecture](./hybrid-architecture/INDEX.md) | Decisions, GI/lighting leak fixes, measured perf findings and plans for `src/hybrid`. Also the decision to move characters onto Bevy's PBR pipeline. | Before changing `src/hybrid`, adding a render stage, or chasing a light leak or trace-pass cost. |

## External research: how the techniques work (source-verified where stated)

| Domain | What it covers | Read when |
|---|---|---|
| [Human biomechanics — Winter (book digest)](./biomechanics-winter/INDEX.md) | Chapter/section/subsection digest of Winter's *Biomechanics and Motor Control of Human Movement* (2009): segment parameters, inverse and forward dynamics, 3D rotations, energetics, muscle and EMG, balance synergies, and a full measured walking stride, each note linking its PDF pages and its relevance to `src/character/anim`. | Before designing gait, balance, start/stop transitions, ragdoll mass/torque/actuation, or when a motion needs a real-human reference. |
| [Bevy rendering (0.19)](./bevy-rendering/INDEX.md) | Source-verified internals: extraction, camera-driven scheduling, phases, materials and shaders, PBR and lighting, post-processing, 2D/UI/gizmos. | Before any change to rendering code, or to check a Bevy rendering claim against source. |
| [3D signed distance fields](./sdf-3d/INDEX.md) | SDF math, primitives and operators, sphere tracing, shading, mesh ↔ SDF conversion, production case studies, and the state of the art. | Before raymarching, SDF modeling, baking a mesh to an SDF, or debugging raymarch artifacts. |
| [3D Gaussian splatting](./3dgs/INDEX.md) | The 3DGS primitive, training, the tile rasterizer, artifacts, compression, and the state of the art. | Before evaluating or building a splat capture, training or rendering path. |
| [SDF + 3DGS + Bevy integration](./sdf-3dgs-bevy-integration/INDEX.md) | **Archived design.** It used an SDF as the world model, baked it to splats, and rendered those as a native Bevy main-pass citizen. It was built, then removed (splat far-tier deleted 2026-09-06). The domain INDEX keeps the lessons that still hold. | Before reviving splats or SDF→splat baking. Also read it for Bevy pass-ordering and depth-write lessons for custom phases. |
| [Compute shaders](./compute-shaders/INDEX.md) | The GPU execution model, performance practice, CPU↔GPU data flow, and Bevy compute integration. | Before writing or tuning a compute pass, choosing workgroup sizes, or adding GPU readback or timing. |
| [AABBs & spatial acceleration](./aabb-acceleration/INDEX.md) | The slab test, the grid/octree/kd/BVH landscape, Bevy's AABB primitives, and CPU vs. GPU placement. | Before writing a ray/box test or choosing an acceleration structure. |
| [Hierarchical volumes](./hierarchical-volumes/INDEX.md) | BVH build and traversal in depth, the VDB/SVO family, an occupancy first-pass design, and Bevy culling primitives. | Before building a culling/occupancy hierarchy or writing hierarchy traversal in WGSL. |
| [Analytic intersections](./analytic-intersections/INDEX.md) | Exact ray intersection for quadrics and CSG intervals. **Retired from the live renderer** after measuring 5–6× the cost of SDF marching. | Only for non-per-frame uses (picking, collision) or when reconsidering exact intersection. |
