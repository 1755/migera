# Tag registry

The controlled vocabulary for the `tags:` field of every knowledge note. A
tag that is not a row in one of the tables below fails
`python3 tools/kb.py lint`. Rules for using tags are in
[AGENTS.md](./AGENTS.md#tags); this file only lists the tags themselves.

**Tags are cross-cutting facets, not a second folder tree.** The folder a
note lives in already says its home domain. A tag answers "what else would
someone search for and expect to find this note?" Use 2–6 per note. Prefer
an existing tag even if it is a little broad. Add a new tag only when at
least **three** notes (existing or planned) would carry it. Add it here in
the same commit, with a one-line meaning.

Search: `python3 tools/kb.py find <tag> [<tag>...]` (AND), `--any` (OR),
`python3 tools/kb.py tags` (usage counts). Raw grep works too, because tags
are a block list with one per line: `rg -l '^  - raymarching$' docs/knowledge`.

## Domains

| Tag | Meaning |
|---|---|
| `sdf` | Signed distance fields: math, modeling, evaluation. |
| `3dgs` | 3D Gaussian splatting: the representation, training, rendering. |
| `bevy` | Bevy engine APIs and internals (currently 0.19.x). |
| `wgsl` | WGSL shader language specifics and pitfalls. |
| `gpu-compute` | Compute shaders, dispatch, workgroups, GPU execution model. |
| `character-animation` | Skeletal/procedural animation of characters. |
| `physics` | Rigid bodies, joints, constraints, collision (incl. avian). |
| `math` | Geometry, linear algebra, quaternions, calculus used by the code. |

## Rendering techniques

| Tag | Meaning |
|---|---|
| `raymarching` | Sphere tracing / marching a ray through a distance field. |
| `rasterization` | Triangle or splat rasterization, tile binning, sorting. |
| `ray-tracing` | Exact ray–primitive intersection, hardware/software RT. |
| `lighting` | Direct lighting, BRDFs, light types, PBR shading. |
| `shadows` | Hard/soft shadows, shadow maps, shadow rays, AO. |
| `global-illumination` | Indirect light: DDGI, radiance cascades, probes, bounce. |
| `materials` | Material models, texturing, material blending, UV-less projection. |
| `post-processing` | Full-screen effects: bloom, tonemapping, DoF, AA, SSR. |
| `temporal` | Temporal accumulation/reprojection, TAA, history buffers. |
| `render-pipeline` | Render graph/schedule, passes, phases, extraction, views. |

## Geometry and data structures

| Tag | Meaning |
|---|---|
| `spatial-acceleration` | BVHs, grids, octrees, VDB, empty-space skipping. |
| `bounding-volumes` | AABBs, OBBs, slab tests, bounds math. |
| `culling` | Frustum/occlusion/visibility culling, occupancy passes. |
| `lod` | Level of detail, clipmaps, resolution scaling. |
| `mesh-conversion` | Mesh ↔ SDF/splat conversion, baking, extraction. |
| `csg` | Boolean/smooth combination of shapes, interval CSG. |
| `primitives` | Catalogs of shapes and their distance/intersection functions. |

## Pipeline and assets

| Tag | Meaning |
|---|---|
| `assets` | Asset formats and import: glTF/GLB, scenes, pose files. |
| `baking` | Offline or incremental precomputation (SDF grids, splats, probes). |
| `streaming` | Large-world streaming, invalidation, incremental updates. |
| `compression` | Size reduction of scene data (quantization, pruning, SH). |
| `ecs` | Bevy ECS patterns: components, systems, schedules, extraction. |

## Character animation topics

| Tag | Meaning |
|---|---|
| `rig` | Skeleton structure, bind poses, bone naming, the glTF rig. |
| `retargeting` | Mapping poses/deltas between rigs and bind frames. |
| `ik` | Inverse kinematics: leg/arm IK, foot locking, reach. |
| `locomotion` | Gait, walk cycles, phase oscillators, ground contact. |
| `ragdoll` | Physics-driven bodies, PD control, strength blending. |
| `springs` | Damped harmonic oscillators, inertialization, smoothing. |
| `poses` | Authored pose data and its verification. |

## Gameplay

| Tag | Meaning |
|---|---|
| `camera` | Gameplay camera rigs: follow, orbit, collision/occlusion, framing, lock-on, input feel. |

## Human biomechanics (research literature)

| Tag | Meaning |
|---|---|
| `biomechanics` | Mechanics of real human movement from the biomechanics literature (gait data, segment models). |
| `anthropometry` | Body-segment lengths, masses, centers of mass, moments of inertia. |
| `inverse-dynamics` | Joint reaction forces and moments computed from motion + ground reaction force; link-segment models. |
| `energetics` | Mechanical work, energy, power and efficiency of movement. |
| `muscle` | Muscle mechanics, motor units, force-length/velocity, EMG and muscle models. |
| `balance` | Posture and balance control: COM vs. COP, inverted pendulum, gait initiation/termination. |
| `signal-processing` | Filtering, smoothing, differentiation, spectra, correlation, ensemble averaging of motion signals. |

## Engineering concerns

| Tag | Meaning |
|---|---|
| `performance` | Measured cost, bottlenecks, optimization results (incl. null results). |
| `numerics` | Floating-point precision, approximations, stability, integration. |
| `correctness` | Real bugs, leaks, wrong results, and their root causes. |
| `testing` | How to write tests that can actually fail; test design traps. |
| `debugging` | Diagnosis methodology, live inspection (BRP, gizmos), bisecting. |
| `verification` | Proving a change works: screenshots, measurements, A/B. |
| `tooling` | Dev tools, scripts, editors, profilers, the anim studio. |

## Knowledge kind (use sparingly — `type:` already says most of this)

| Tag | Meaning |
|---|---|
| `hybrid-renderer` | About migera's own `src/hybrid` renderer specifically. |
| `prior-art` | How another engine/game/paper solved it; not migera code. |
| `state-of-the-art` | Survey of current research/production practice (dated!). |
| `case-study` | A detailed account of one shipped system. |
| `troubleshooting` | Symptom → cause → fix maps. |
| `integration` | Wiring one system into another (e.g. into Bevy's pipeline). |
