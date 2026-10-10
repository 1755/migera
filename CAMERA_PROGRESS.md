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

With collision (`--collision`): a headless avian world of 500 static boxes; the full
pipeline plus the boom sweep, shoulder slide, one feeler, the ceiling sweep and overlap
tests per rig.

| Cameras | p50 | p99 | per-camera p50 |
|---:|---:|---:|---:|
| 1 | 0.0092 ms | 0.0749 ms | 9.2 µs |
| 64 | 0.899 ms | 1.183 ms | 14.0 µs |

## Log

### 2026-10-10 — P2: collision and occlusion

**What was built.**
- **`src/camera/probe.rs`.**
  - `CameraProbe` asks two things: sweep a sphere, and does a sphere overlap anything.
  - `AvianProbe`: `cast_shape_predicate` with `ignore_origin_penetration`, and
    `shape_intersections_callback`, filtered by a layer mask and an ignore predicate.
  - `SdfProbe`: boxes from `src/sdf`, sphere-traced with the same origin-contact rule.
  - `NoProbe`.
- **`src/camera/collision.rs`: `CollisionState::resolve`.**
  - Start: a free start chain; the shoulder slide with automatic shoulder swap.
  - Boom: a near-plane-sized sweep (`near_plane_radius`), with collision vs occlusion
    decided by the eye's position *and its own path*, a minimum occlusion time, and a
    hold then ease-out.
  - Lyra's feeler table, round-robin, as soft caps.
  - Ceiling: a pitch cap fed back to the orbit, which eases under it.
  - Fallback: hysteretic high view at 70° or near-vertical, whichever reaches further.
  - Target fade.
- **`src/physics_avian/layers.rs`.** `TERRAIN_LAYER` (now shared with
  `physics_character_playground`), `CAMERA_TRANSPARENT`, `RAGDOLL_LAYER_POOL` and
  `CAMERA_BLOCKERS`.
- **ECS.**
  - `CameraSet::Collide` runs `collide_cameras`: avian when the app has it, an empty world
    otherwise.
  - New components: `CameraResolvedPose`, `CameraIgnore`, `CameraTarget::exclude`.
  - `CameraView` gains `distance`, `desired_distance`, `target_fade` and `fallback`.
- **Pipeline and tooling.**
  - `CameraRig::resolve` / `step_resolved`, `CameraTrace::replay_resolved`.
  - `harness::measure_collision`: inside frames, blocked frames, longest blocked run.
- **Orbit.** Auto-recentre only while moving *away* from the camera.
- **Playground.**
  - `keep_out_of_walls`: the character's root motion is swept as a capsule and slid along
    the static geometry. It's a stand-in for a character controller.
  - Teleport fixed: it now moves `locomotion.position`; P1's moved the `Transform`, which
    root motion overwrote.
  - Scripts `wall`, `along`, `corridor`.
  - The HUD shows boom, fade and fallback; the inset draws the desired and resolved booms.
- **`camera_bench --collision`.**

**Gates, all passing (64 camera tests; full lib 1391):**
- **Pure** (`SdfProbe`):
  - swinging into a wall snaps in within one frame, holds 12 frames, then eases out to
    full;
  - a 50 ms occlusion is ignored, a lasting one pulls in;
  - penetration snaps even with a 10 s occlusion wait;
  - feelers pull in while the main sweep is clear;
  - an embedded pivot starts from a free height;
  - a shoulder slides in and the boom keeps its length along the wall;
  - a blocked shoulder swaps to the open side (control without swapping: pinned);
  - a 1 m roof caps pitch at `asin(0.85/3)`;
  - the fallback waits for a sustained short boom and releases with hysteresis;
  - fade near the character;
  - near-plane radius 0.131 m at Bevy's defaults;
  - a 2 cm wall stops the sweep;
  - a ceiling cap eases pitch rather than snapping;
  - walking sideways or toward the camera never recentres.
- **Scenario.** 40 s through an SDF copy of the playground (corridor, along the wall,
  past pillars and a 2 cm fence) with the camera orbiting and nodding throughout.
  - **Inside frames: 0**, longest blocked run 1 frame (bound: occlusion time + 2 frames).
  - Without collision, measured on the same geometry: 171 inside frames and a 2.95 s
    blocked run.
- **Headless avian.**
  - A full turn against a real wall never puts the eye inside it, checked by avian
    directly; the boom pulls in and eases out.
  - Camera-transparent, `CameraIgnore`, ragdoll-layer and `exclude`d colliders never
    shorten the boom; a plain box does.

**Sabotage checks.**
- The ECS probe swapped for `NoProbe`: both avian tests fail.
- The ignore predicate disabled: boom 0.35 of 3.2.
- Hold time 0: held 0 frames.
- Skin removed together with the ignore-on-contact rule: boom collapsed to 0.

**Visual check** (Xvfb `:97`, lavapipe, claim stated first):

| Shot | Claim | Result |
|---|---|---|
| `along` f600, camera turned along the long wall | Over the open shoulder, boom > 1 m, no high view | yes: boom 1.39/3.2 m, high 0.02, wall along the right |
| `wall` f600, camera swung to the wall side | High view or pulled in; no wall interior | yes: near-vertical high view, the wall's top at the frame's edge |
| `corridor` f660, under the roof looking down | Pitch held near 15° by the roof, no roof interior | yes: pitch 15.3° under 2.5 s of look-down input; boom 2.33 m (side feelers) |

**Found by looking, and fixed** (each became a test):
1. **The character spiralled.** Camera-relative "walk left" with auto-recentre toward the
   heading turned "left" with the camera, so the character walked in a spiral and ended
   up east of where it started. Auto-recentre now needs travel within ~60° of
   straight-away. This changed the golden trace, which was regenerated.
2. **Boom 0 m and pitch flattened to 7.7° beside a wall.** The shoulder slid into contact,
   and every later sweep, avian and SDF alike, read a hit at distance 0, including the
   ceiling sweep. Fixed with a skin and the ignore-on-contact rule. Lesson:
   `docs/knowledge/gameplay-camera/a-sweep-from-contact-reads-as-a-hit-at-zero.md`.
3. **The boom was still pinned with the shoulder against the wall,** even though the
   shoulder kept 64% of its offset: the boom angled into the wall. This led to the
   shoulder swap, triggered by the *boom's* room.
4. **The high view sat half a metre over the head,** because its 70° boom ran into the
   wall behind. It now also tries a near-vertical boom and takes the longer.
5. **Swinging across a wall counted as occlusion.** The eye ended in free space beyond
   the wall, so the camera waited out the grace time behind it. The eye's own path now
   counts as collision.

A live `wall` run that keeps orbiting was read over BRP (`CameraRigState`). It showed the
boom legitimately pinned while the camera sweeps into the wall: the hold resets every
frame and the swap is mid-transition. That's correct for a swing; the static scripts
check the settled cases.

**Measured.** See the baseline table: collision costs ~9–14 µs per camera, against
0.2 µs for the rest.

**Not built in P2** (moved later):
- hill lift (rise over terrain);
- velocity whiskers;
- per-mode occluder policy and fading, which need a dithered material;
- a test for a moving platform. The anchor's offset-from-target spring covers it in
  principle; there is no platform in the playground yet.

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
