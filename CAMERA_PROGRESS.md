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

## Baseline

`cargo run --release --example camera_bench -- --cameras N --frames 3000`
(Linux 6.18, release). One frame per rig: mode stack, anchor, orbit, rig
pose; no collision yet. Each rig follows a target round a circle with the
stick moving and a mode switch every 2 s.

| Cameras | p50 | p99 | per-camera p50 |
|---:|---:|---:|---:|
| 1 | 0.0014–0.0019 ms | 0.0015–0.0024 ms | 1.4–1.9 µs (fixed overhead dominates) |
| 64 | 0.0134 ms | 0.0214 ms | 0.21 µs |
| 1000 | 0.195 ms | 0.315 ms | 0.20 µs |

## Log

### 2026-10-10 — P1: plugin, input, walker bridge, playground

**What was built.**
- **`src/camera/plugin.rs`: `ThirdPersonCameraPlugin`.**
  - `CameraSet::Input` runs in `Update`.
  - In `PostUpdate`, `CameraSet::{Target, Rig, Collide, Effects, Write}` are chained after
    `CameraTargetSources`, which is ordered after physics writeback and
    `RagdollSet::ReadBack`. The chain runs before transform propagation.
  - Every component is registered for reflection.
- **`components.rs`.**
  - `ThirdPersonCamera` requires the rest: hello-world is
    `(Camera3d, ThirdPersonCamera::follow(e))`.
  - The other components: `CameraTarget`, `CameraTargetState`, `CameraModeRequests`,
    `CameraGoals`, `CameraRigState`, `CameraDesiredPose`, `CameraView`, `CameraRecorder`.
  - The pure `update_latch` holds `control_yaw` across a cut while a direction is held;
    after 2 s it eases to the view.
- **`device.rs`.** `CameraDeviceInput` maps mouse, keys and gamepad:
  - mouse look always, while a button is held, or while the cursor is grabbed;
  - scroll and D-pad zoom;
  - R, middle mouse or R3 recentres; Tab or L3 swaps the shoulder.
  - It skips itself without Bevy's input plugin.
- **`bridge.rs`.** Fills `CameraTargetState` (grounded, facing) from `WalkerState`. The
  position comes from the root `Transform`.
- **Pipeline additions.**
  - Shoulder swap: a sprung side.
  - A per-target pivot offset on `TargetSample`.
  - `CameraInput::consume` and `move_held`.
- **`examples/camera_playground.rs`.**
  - A walker on a field with a wall, pillars, a roofed corridor, a low tunnel and crates.
    All have static colliders, ready for P2.
  - WASD/stick moves relative to `CameraView::control_yaw`. Run, jump, combat toggle,
    recentre, shoulder swap, and a teleport to make a cut.
  - Scripts `orbit|walk|tour`; `--debug-view` (top-down inset; the rig drawn on its own
    gizmo layer).
  - `--record` (F9 or on exit) and `--replay PATH`, which prints metrics plus the target's
    vs the eye's peak acceleration.

**Gates, all passing (44 camera tests; full lib 1371):**
- `the_camera_follows_a_ragdoll_read_back_in_the_same_frame`: a target moved in
  `RagdollSet::ReadBack` is where the camera's pivot is, the same frame.
- `camera_input_turns_the_camera_with_no_devices_and_is_consumed`.
- `a_third_person_camera_without_camera3d_is_left_alone`: the bare shadow-view `Camera`.
- `a_cut_while_moving_latches_the_control_yaw_in_the_world`, plus 4 pure latch tests.
- `two_cameras_take_separate_inputs`.
- `switching_target_hands_over_without_a_cut`.

**Sabotage checks.**
- Camera chain ordered before `ReadBack`: it reads the target a frame late (pivot x 0 vs
  target x 10).
  - The first sabotage attempt, moving only `CameraTargetSources`, still passed. The chain
    and the read-back were left unordered, and the scheduler happened to run the read-back
    first.
- Latch disabled: both latch tests fail.

**Visual check** (Xvfb `:97`, lavapipe, `--step-seconds 0.0166667`; claim stated first
each time):

| Shot | Claim | Result |
|---|---|---|
| `walk`, frame 150 | Character seen from behind walking away (−Z), a little left of centre (right shoulder), view from above, level horizon | yes |
| `walk`, frame 330, after a right camera swing | Character turned with the camera, seen from behind again; view rotated | yes (yaw −116°, crates ahead) |
| `orbit`, frame 200 | Standing character in profile, centred, level | yes |
| `tour` + `--debug-view`, frame 500 | "mode combat", boom ≈ 2.5 m, closer, clean main view, rig in the inset | yes (boom 2.53 m) |

Found and fixed along the way, all in the example:
- The HUD rendered in the inset; it now has `UiTargetCamera`.
- `°` and `·` were missing from the font.
- The eye marker and forward arrow, drawn from the gameplay camera itself, smeared across
  the view. They moved to a gizmo layer only the inset renders.

**Measured.**
- `tour` replay: 530 frames, max roll 1.5e-8.
- The walker's root peaks at 79 m/s² of acceleration from the gait.
- The eye peaks at 61 m/s² at frame 169, which is the scripted stick look being released.
  That is player input on a 3.2 m boom, not following.
- `camera_bench` is unchanged: 0.21 µs per camera at 64, 0.20 µs at 1000.

**Known limits (for P2+).**
- The character walks through the geometry (flat ground, no character collision), and
  the camera goes through walls: collision is P2.
- There is no `.camera.ron` profile yet; the config is inline.

### 2026-10-10 — P0: pure pipeline, trace replay, scenario harness

**What was built.**
- **Math moved up.** `src/character/anim/math` is now `src/math`. `character::anim::math`
  remains as a re-export, so no animation code changed.
- **New math.**
  - `src/math/angle.rs`: wrapping, shortest-arc `damp_angle`, `damp`, radial deadzone,
    stick curve, soft limit.
  - `spring_scalar_tracking` / `spring_vec3_tracking`: exact follow of a goal that moves
    linearly over the frame.
  - `SpringParams` derives `Reflect`.
- **`src/camera`: the pure stages.**
  - `stack` is the push-only mode stack, with smoothstep weights and a re-push that keeps
    its contribution.
  - `anchor` is the pivot:
    - follow and leashes: tracking spring, horizontal and vertical leashes;
    - jumps and cuts: air deadband with a ratchet, teleport cut;
    - inertialized anchor rebase; look-ahead.
  - `orbit` handles look and goals:
    - input: exact stick integral, mouse not scaled by `dt`, soft and hard pitch limits;
    - goals own axes by priority, with a decaying player offset;
    - recentring: button, and delayed auto-recentre on the part of the frame past the delay;
    - zoom.
  - `rig` is pitch curves, `ModeParams` exploration/combat, and the eye pose.
  - `pipeline` (`CameraRig`, `CameraFrame`, `CameraConfig`) chains them over plain data.
- **Tooling.**
  - `trace`: `CameraTrace`, `.camtrace.ron`, one frame per line, plus golden traces.
  - `harness`: time-parameterised `Scenario` sampled at any rate, and `Metrics`.
  - Golden trace at `tests/golden/camera/walk_turn_combat.camtrace.ron`.
  - `examples/camera_bench.rs`.

**Gates, all passing (35 camera tests and 4 new spring tests, inside a full lib run of 1361):**
- The eye agrees at 30/60/144 Hz every 1/6 s for 6 s, within 2 cm; measured
  sub-millimetre. The scenario includes a walk, a turn, stick looks and two mode blends.
- No roll anywhere in the scenario: `max_roll` < 1e-5.
- Pitch stays inside its limits under full stick for 10 s.
- Angle damping takes the short way across ±π.
- No recentring before the delay or while standing still.
- A hop inside the air band leaves the pivot height unchanged; a 4.9 m fall is tracked.
- The leash caps lag at a 12 m/s sprint. A counter-test without the leash trails further.
- The golden trace replays within 1 cm, and a trace round-trips through RON and replays
  bit-identically.
- Paused clock: the stick still turns the camera by more than 1 rad, and the pivot does
  not move.
- A re-pushed layer keeps its contribution, to within 1e-4 m of distance.
- A layer under a full-weight top is dropped.
- An anchor switch of 3 m: under 0.06 m of pivot motion in its frame, and under 50 m/s²
  of acceleration.
- A mode blend's eye acceleration stays within 1.15 × the smoothstep bound `6Δ/T²`.

**Sabotage checks.** Each fix was undone and its test had to fail:
- staircase follow → 3.95 cm frame-rate gap, golden trace fails;
- linear blend weight → 98.8 m/s² against a bound of 28.4, plus the spike and re-push tests;
- Euler stick → 3.54 cm, and held yaw 0.668 vs 0.651 rad;
- push before step → 4.88 cm mid-blend;
- whole-frame recentre → 0.787 vs 0.780 rad.

**Dead ends and lessons.**
- Exact dampers alone still left the eye 2.9 cm apart between 30 and 144 Hz. The causes
  were event order, input sampling time and mid-frame thresholds.
- Two tests passed under sabotage at first. One sampled at whole seconds, outside the
  blends; the other compared only after ramp errors had cancelled.
- Written up as `docs/knowledge/gameplay-camera/frame-rate-independence-needs-exact-events-and-thresholds.md`.
- The first rebase put the restarted follow *on* the goal instead of at its steady trail.
  The stiff spring then yanked it 0.2 m, at about 1200 m/s² for a 3 m switch.
- The decision note's "blend anchors as weighted points" became an inertialized hand-off.

**Next: P1.** `ThirdPersonCameraPlugin`, components and systems, mouse/gamepad mapping,
the walker bridge, and `examples/camera_playground.rs`, with an Xvfb visual check.

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
