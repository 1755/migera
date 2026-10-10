---
title: Third-person camera design
description: "Design of migera's planned third-person camera (src/camera): a pure-function pipeline (anchor, orbit, rig, layer stack, post-blend collision, effects), per-stage ECS components as extension points, a latched control yaw, real vs virtual time, RON profiles, trace replay. Read before building the camera."
type: design
status: draft
tags:
  - camera
  - ecs
  - physics
  - testing
updated: 2026-10-10
code:
  - src/camera
  - src/math
  - src/character/anim/walker.rs
  - src/character/anim/ragdoll_plugin.rs
sources:
  - "Research notes in this folder"
  - "Design review of the first draft, 2026-10-10"
aliases:
  - ThirdPersonCameraPlugin
  - CameraView
  - control_yaw
  - CameraProfile
  - resolve_boom
  - CameraTrace
---

# Third-person camera design

Contents: [Goal](#goal) · [Pipeline](#pipeline) · [Data model](#data-model) ·
[Scheduling](#scheduling) · [Collision](#collision) · [Lock-on](#lock-on-and-framing) ·
[Profiles and DX](#profiles-and-developer-experience) · [Testing](#testing-strategy) ·
[Open questions](#open-questions) · [Status](#status)

A reusable `ThirdPersonCameraPlugin` in `src/camera`, built as a fixed pipeline
of small stages. Each stage is a pure function over plain structs and owns
specific degrees of freedom. Stages talk only through per-stage ECS
components. The architecture rests on
[one rig with blended layers](./one-rig-with-blended-layers-over-blending-virtual-cameras.md).
The behaviours come from the research in this folder.

## Goal

An action-RPG camera that:
- follows a root-motion character smoothly at any frame rate;
- never shows the inside of a wall or of the character;
- keeps the character visible without fighting the player;
- frames combat and lock-on;
- moves between modes with no pops;
- is tuned from data;
- can be reproduced from a recorded trace when it misbehaves.

Character animation and movement are out of scope. The camera publishes the yaw a
player controller needs, and nothing more.

## Pipeline

Each stage below owns its outputs; no later stage writes them.

1. **Input** (`Update`). Devices are mapped to a `CameraInput` component. Tests, replay
   and AI write the same component.
2. **Anchor.**
   - The pivot follows a goal point (target plus eye-height offset) with a critically
     damped spring. It is solved exactly over the interval the goal moved in, so it does
     not stick to a moving base, and its lag is the same at any frame rate.
   - Horizontal: a hard leash, so the camera can't trail further back at speed.
   - Vertical: its own leash, so a fall can't leave the character below the frame.
   - Airborne: a deadband above take-off, so jumps aren't followed but falls are.
   - Look-ahead along velocity, sprung and clamped.
   - An anchor switch is inertialized (see the decision note).
   - A teleport snaps everything and sets `cut_this_frame`.
3. **Orbit.**
   - Mouse delta is integrated without `dt`. Stick rate goes through deadzone → S-curve →
     acceleration, then is integrated.
   - Soft pitch limits ease in, then a hard clamp.
   - Recentre behind movement after a delay, only while moving *away* from the camera.
     Strafing would otherwise turn "sideways" with the camera and spiral the character;
     moving toward the camera would flip the view.
   - Goal arbitration (lock-on, dialog, assist, recentre): one owner per degree of freedom,
     player offset decays under a goal.
4. **Rig.** Evaluates the stack's blended parameters at the current pitch (distance,
   height and FOV as `PitchCurve`s, shoulder side sprung) into a `CameraDesiredPose`.
5. **Collide.** `resolve_boom`, run once on the blended pose; see [Collision](#collision).
6. **Effects.** Trauma² rotational shake and FOV kick, scaled by a comfort setting. This
   is the only stage that may add roll.
7. **Write.** `Transform`, `Projection` FOV and the public `CameraView`.

### Time

- `CameraClock { real_dt, virtual_dt }` is gathered once per frame.
- **Real time:** look input, recentre timers, blends and effects. A paused or hit-stopped
  game must still let the player look around.
- **Virtual time:** follow springs and look-ahead, so the pivot freezes with the world.
- The profile can override per stage group.
- **Within a frame:** existing blends advance over the frame before that frame's mode
  requests apply. A threshold on accumulated time (the recentre delay) acts only on the
  part of the frame past it. See
  [frame-rate independence needs exact events and thresholds](./frame-rate-independence-needs-exact-events-and-thresholds.md).

### Damping

All damping is closed-form: `SpringParams` / `spring_vec3` / `decay_exponential`; see
[damping](./camera-damping-is-exponential-not-a-per-frame-lerp.md).

## Data model

All components derive `Reflect`, so BRP can inspect them.

**Camera entity** (`Camera3d`; every query is `With<Camera3d>`, because the shadow view
carries a bare `Camera`):
- `ThirdPersonCamera { target, config }` requires the rest, so the hello-world is
  `commands.spawn((Camera3d::default(), ThirdPersonCamera::follow(player)))`.
- `CameraInput { look_delta, look_stick, zoom, recenter, lock_on, switch, shoulder_swap,
  move_held }` is the only input contract.
  - The camera consumes the one-shot parts each frame.
  - `move_held` belongs to the game's movement controller.
  - `CameraDeviceInput` opts a camera into mouse, keyboard and gamepad mapping. Without
    it, a test, replay or AI drives the camera.
- `CameraInputSettings`, a component so each split-screen player has their own:
  per-device sensitivity and curve, invert, deadzone, acceleration, shake scale.
- `CameraModeRequests` and `CameraGoals`: this frame's mode switches and orbit goals,
  cleared once used.
- Per-stage state and outputs:
  - `CameraClock`;
  - `CameraRigState`, which wraps the pure `CameraRig` (anchor, orbit, stack);
  - `CameraDesiredPose`;
  - `CameraResolvedPose`, from P2.
- `CameraView`, the public output: eye, rotation, fov, pivot, `view_yaw`, `pitch`,
  `control_yaw` with its latch, `cut_this_frame`, and `control_forward`/`control_right`.
  P2 and P5 add distances and `target_fade`.
- `CameraRecorder` records every consumed frame as a `CameraTrace`.

**Control yaw latch.** A player controller maps the stick against `control_yaw`, not
`view_yaw`. `control_yaw` holds its value across a cut, or a lock-on swing, for as long
as the stick stays held, and re-syncs on release or after a timeout. "A cut must not
remap controls" is a mechanism here, not a guideline.

**Extension points are the per-stage components.** A game system ordered
`.after(CameraSet::Rig).before(CameraSet::Collide)` may edit `CameraDesiredPose`. That is
the ECS form of a Cinemachine extension or an Unreal camera modifier, with no trait
objects.

**Target and world components:**
- On the target: `CameraTarget { pivot_offset, safe_offsets, exclude }`,
  `CameraTargetState { velocity, grounded, facing_yaw }`, and `CameraContext` flags or
  direct `ModeRequest { id, priority, blend }`s.
- On other entities: `LockOnTarget { radius, priority, aim_offset }`, `CameraFadeable`,
  `CameraVolume` (pushes a mode request while inside), `CameraIgnore`, `CameraOverride`.

**Bridge.** `bridge.rs` is the only file that knows the walker. It fills
`CameraTargetState` from `WalkerState`, and reads position from the root `Transform`,
because `follow_the_fallen_body` copies only x/z into `locomotion.position`.

## Scheduling

1. `CameraSet::Input` runs in `Update`.
2. In `PostUpdate`, a named set `CameraTargetSources` contains every system that last
   moves a target: `RagdollSet::ReadBack` today, and any physics-interpolation set later.
3. The chain `Target → Rig → Collide → Effects → Write` runs after
   `CameraTargetSources` and before `TransformSystems::Propagate`.
   - `Target` gathers clocks and runs target bridges.
   - `Rig` runs stack, anchor, orbit and rig as one system over the pure `CameraRig`.
     Finer sets would only be worth adding when a consumer needs to step between them.

The chain reads the target's `Transform`, which is final for the frame. `GlobalTransform`
is still last frame's at that point.

A test pins the order: `the_camera_follows_a_ragdoll_read_back_in_the_same_frame`. Its
sabotage must order the camera chain *itself* before `ReadBack`. Moving only
`CameraTargetSources` leaves the two unordered, and the scheduler happened to run the
read-back first, so the test still passed.

## Collision

`resolve_boom` implements
[the collision techniques note](./camera-collision-and-occlusion-techniques.md):

0. **Find a free start.** Use the pivot, or if it is embedded, the first free point of
   `safe_heights` above the target's root.
1. **Slide the shoulder**, a skin short of any hit.
   - Swap to the other shoulder while this side's boom has under half its length and the
     other side gives clearly more; swap back at 90%.
2. **Main sweep, shoulder → eye**, using a sphere at least as large as the near-plane
   half-diagonal (derived from `Projection`).
   - Snap in when the eye would be inside geometry, or its own move crossed some.
   - Otherwise it is occlusion: pull in after `min_occlusion_time`.
3. **Ease out.** Hold, then ease out on a half-life.
4. **Feelers.** Lyra's table, one re-traced per frame. They act as soft caps; yaw swing
   is off, because intent wins.
5. **Ceiling.** An up sweep gives a pitch cap; the orbit eases under it next frame.
6. **Avatar fade.** `target_fade` comes from the eye-to-pivot distance.
7. **Fallback.** A hysteretic high view, at the configured pitch or near-vertical,
   whichever reaches further.

Not built yet:
- hill lift (rise over terrain instead of shortening);
- velocity whiskers;
- per-mode `OccluderPolicy { PullIn, Fade, PullInThenFade }`, which waits for a fade
  renderer.

**Probes.** Collision goes through a `CameraProbe` trait:
- `SdfProbe` for unit tests, built from `src/sdf` shapes. A sphere cast against an SDF is
  sphere tracing minus the radius, so boxes, capsules and CSG come free.
- `AvianProbe` for runtime: `SpatialQuery::cast_shape` with a blocker mask, excluded
  entities and a `CameraIgnore` predicate.

**Layers** live in `src/physics_avian/layers.rs`:
- `TERRAIN_LAYER`;
- `CAMERA_TRANSPARENT` (foliage, thin props, characters);
- `RAGDOLL_POOL`.

The default blocker mask excludes the last two.

## Lock-on and framing

- **Select.** Candidates inside a cone around the camera forward, within range, with line
  of sight. Score by angle, distance and priority.
- **Switch.** A flick (stick crossing a high threshold after being near rest) picks the
  candidate whose screen-space offset best matches the flick direction.
- **Break.** Beyond a distance with hysteresis, after line of sight has been lost for a
  grace time, or when the target despawns.
- **Frame.** Emit a yaw/pitch goal toward the target. Pull back distance and FOV from the
  bounding sphere of the player and all locked targets (target group). Aim at a weighted
  midpoint inside a dead zone.
- **Character facing** belongs to the controller, not the camera.

## Profiles and developer experience

**`CameraProfile`** is a `.camera.ron` asset holding:
- named modes with `inherits`;
- selection rules (`when context.lock_on → "combat"`);
- goal priority;
- anchor, orbit, collision, lock-on and effects parameters.

Behaviour:
- `CameraProfile::default()` is embedded, so hello-world needs no file.
- The loader follows `PoseAssetLoader` in `src/character/anim/asset.rs`.
- Validation errors name the mode and the field.
- Hot reload keeps rig state, and keeps the old profile when the new file is invalid.

**Tuning values live only in the profile.** Seeds come from Gothic, Skyrim and Lyra; see
the research notes.

**Debugging tools:**
- The `camera_debug` feature adds gizmos: safe-pivot chain, pivot, desired vs resolved
  eye, feelers coloured by hit, hit normals, lock-on candidates with scores. It also adds
  an egui tuning panel that saves back to RON.
- `CameraTrace` (`.camtrace.ron`) records per-frame
  `(real_dt, virtual_dt, CameraInput, CameraTargetState)` from the playground, and
  replays it into the pure pipeline. A camera bug a player felt becomes a deterministic
  test from one file.

## Testing strategy

- **Pure stages are tested without a `World`.** A scenario harness runs a trajectory and
  input script through the pipeline and reports metrics per frame and in aggregate:
  - maximum eye jerk;
  - line-of-sight ratio;
  - occluded frames;
  - frames with the eye inside a collider;
  - maximum roll.
- **Frame-rate independence** compares positions at common timestamps, sampling a
  time-parameterised input at 30, 60 and 144 Hz.
- **Golden traces** replay to within 1 cm.
- **Collision runs at two levels:** analytic (`SdfProbe`, fast) and a headless avian
  sweep (orbit 360°, corridor, back to wall, under a ceiling) asserting zero
  inside-collider frames and a line-of-sight ratio of 1.
- **Every behaviour test is sabotage-checked.** For example, swap the exponential damper
  for a `lerp(k·dt)`, or move the chain before `ReadBack`, and confirm the test fails.

## Open questions

- **Interiors.** Detect them with space probes that cap distance, or with authored
  `CameraVolume`s, or both? Probes are generic but can flicker in clutter.
- **Occluder fade rendering.** Avatar and occluder fade needs a dithered material on
  Bevy's PBR pipeline. Not designed yet; the camera only emits fade amounts and markers.
- **Feeler yaw swing.** Journey swings yaw away from side occluders. Is it worth offering
  when it fights intent? Decide after playtesting.
- **Mounts.** Mounted riding needs a second anchor and a longer lag. Revisit when mounts
  exist.

## Status

What is built, phase by phase, with test gates and measurements, is in
`CAMERA_PROGRESS.md`.
- **Built (P0–P2):** the pipeline, the ECS plugin, input, collision and occlusion, and
  `examples/camera_playground.rs`.
- **Not built:** profiles, lock-on, effects and library debug tools.

## Related

- [One rig with blended layers](./one-rig-with-blended-layers-over-blending-virtual-cameras.md) — prerequisite: why layers carry parameters, an anchor and a goal.
- [Camera collision and occlusion techniques](./camera-collision-and-occlusion-techniques.md) — deeper: what each `resolve_boom` stage is for.
- [Camera damping is exponential, not a per-frame lerp](./camera-damping-is-exponential-not-a-per-frame-lerp.md) — prerequisite: the only allowed smoothing forms.
- [Frame-rate independence needs exact events and thresholds](./frame-rate-independence-needs-exact-events-and-thresholds.md) — prerequisite: the in-frame timing rules every stage follows.
- [Fifty camera mistakes digest](./fifty-camera-mistakes-nesky-digest.md) — deeper: the behavioural rules the pipeline enforces.
- [Shipped action-game camera behaviours](./shipped-action-game-camera-behaviours.md) — example: lock-on, combat framing and the option set.
- [A jump forward leans out over its toes, and its travel is the root's](../character-animation/ik-and-locomotion/a-jump-forward-leans-out-over-its-toes-and-travels-as-root-motion.md) — applies: root motion is spread across a jump so a following camera doesn't jump.
