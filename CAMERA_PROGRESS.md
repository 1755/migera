# Third-Person Camera — Progress Log

Tracks the build-out of `src/camera`, the gameplay camera plugin. Kept
separate from [PROGRESS.md](./PROGRESS.md) (renderer) and
[CHARACTER_PROGRESS.md](./CHARACTER_PROGRESS.md) (animation) so three
unrelated subsystems don't interleave into one chronology.

The design and its reasons live in the knowledge base. This file holds the
roadmap, its test gates, and dated entries.
- Design: [docs/knowledge/gameplay-camera/third-person-camera-design.md](./docs/knowledge/gameplay-camera/third-person-camera-design.md)
- Decision: [one-rig-with-blended-layers-over-blending-virtual-cameras.md](./docs/knowledge/gameplay-camera/one-rig-with-blended-layers-over-blending-virtual-cameras.md)
- Domain index: [docs/knowledge/gameplay-camera/INDEX.md](./docs/knowledge/gameplay-camera/INDEX.md)

## How to use this file

- **One entry per proven-correct phase, newest first.** "Proven" means:
  - every gate below for that phase passes under `cargo test --release`;
  - each gate is sabotage-checked: it fails with the fix disabled;
  - from P1 on, the camera is visually verified on Xvfb `:97` with lavapipe:
    - shots from the gameplay camera, plus the `--debug-view` top-down gizmo view of the same frames;
    - scenes: corridor, back to wall, under ceiling, hill crest, lock-on circle;
    - the yes/no claim is stated before looking at each shot.
- **Record measured cost** from `examples/camera_bench.rs` (headless, 500
  colliders, 1 and 64 rigs; budget < 50 µs per camera). Never use a windowed
  example's frame time, which is vsync-capped.
- **Record dead ends and null results.**

## Roadmap

| Phase | Builds | Test gates |
|---|---|---|
| P0 | Move `src/character/anim/math` → `src/math` with a re-export shim; `math/angle.rs`; pure anchor/orbit/rig/stack stages; `trace.rs` record/replay; `harness.rs` scenario metrics | identical eye at 30/60/144 Hz from time-parameterised input, compared at common timestamps (sabotage: `lerp(k·dt)`); no roll for any yaw/pitch; pitch within limits under full stick; angle damping takes the short way across the wrap; no recentring before the delay or while standing; a jump inside the deadband doesn't move the pivot, a fall does; the leash caps lag; a replayed golden trace matches within 1 cm; paused clock → look moves, pivot doesn't; a re-pushed layer keeps its weight; layers under a full-weight top are dropped; an anchor switch is velocity-continuous |
| P1 | `plugin.rs`, `input.rs`, `clock.rs`, `bridge.rs`, `examples/camera_playground.rs` (walker on terrain with walls, pillars, corridor, hill, foliage; example-local camera-relative controller; `--record/--replay/--script/--shot/--debug-view`) | target followed the same frame, ragdoll root included (sabotage: run the chain before `ReadBack`); `CameraInput` drives yaw with no devices; the bare shadow `Camera` is ignored; `control_yaw` stays latched across a cut while the stick is held; two cameras with separate inputs |
| P2a | `probe.rs` (`SdfProbe`) + `collision.rs` `resolve_boom` | pull-in is immediate and ease-out waits for the hold; brief occlusion is ignored; penetration snaps despite the timer; feelers cap distance before the main ray hits; start-inside snaps; an embedded safe pivot walks its chain; a ceiling pinch raises the pitch floor; a moving platform keeps the pivot hooked; a thin wall is never passed; the fallback engages only after a sustained short boom |
| P2b | `AvianProbe`, `src/physics_avian/layers.rs` | 20 s headless avian sweep (orbit 360°, corridor, back to wall, under ceiling): zero frames with the eye inside a collider, line-of-sight ratio = 1; ragdoll, transparent and own colliders never shorten the boom |
| P3 | `profile.rs` (`.camera.ron`, inheritance, selection rules, hot reload), `CameraOverride` | eye position and velocity continuous across mode switches; the blended eye stays on the orbit band; RON round-trip; an invalid profile fails at load with mode and field named; reload keeps state and survives a bad file |
| P4 | Lock-on + target-group framing | prefers the target nearest screen centre; a flick right picks the target on the right; breaks after the line-of-sight grace time, not before; distance hysteresis; player and all locked targets stay inside the frustum while circling |
| P5 | Effects (trauma² shake, FOV kick), fade markers, `CameraVolume`, look-ahead, interior and ledge probes | zero trauma → zero shake, trauma² scaling; comfort scale 0 removes shake; no roll with effects off; fadeables marked and unmarked; volume push/pop; the interior distance cap has hysteresis |
| P6 | `camera_debug` feature: gizmos, egui tuning panel, save profile | the playground runs with the feature; a saved profile round-trips |

## Log

### 2026-10-10 — Research and design (no code)

Researched production third-person cameras:
- **Shipped games:** Gothic 1/2 from its shipped scripts and OpenGothic, the Gothic 1 Remake, Journey (Nesky's 50 mistakes), Witcher 3, Skyrim, Souls, AC3, Zelda OoT and TLOU2's options.
- **Engine systems:** Cinemachine 3 docs and source, Unreal SpringArm, Lyra source, UE5's Gameplay Camera System, dolly and Godot.

The findings are distilled into a new KB domain, `docs/knowledge/gameplay-camera/` (8 notes and an INDEX, with a new `camera` tag). The design was reviewed once and revised:
- layers carry an anchor and a goal, not only parameters;
- a latched control yaw;
- data-defined modes;
- per-stage components as extension points;
- real vs virtual time;
- record/replay moved to P0;
- the math module moved to `src/math`.

Unverified areas, marked in the notes: Unreal SpringArm internals, FromSoftware lock-on specifics, Assassin's Creed, Naughty Dog and Breath of the Wild internals, and how shipped games implement occluder dithering.
