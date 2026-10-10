---
title: Engine camera architectures compared — Cinemachine, Unreal, Lyra, Gameplay Camera System, dolly
description: "How Unity Cinemachine 3, Unreal's SpringArm/PlayerCameraManager, Lyra's camera modes, UE5's Gameplay Camera System, Rust's dolly and Godot's SpringArm3D structure a third-person camera, with a feature table and the layered pipeline they share. Read before choosing camera architecture or blending model."
type: research
status: current
tags:
  - camera
  - prior-art
  - state-of-the-art
  - integration
updated: 2026-10-10
sources:
  - "Cinemachine 3.1 manual: https://docs.unity3d.com/Packages/com.unity.cinemachine@3.1/manual/CinemachineBrain.html (also ...CinemachineBlending, CinemachineThirdPersonFollow, CinemachineThirdPersonAim, CinemachineOrbitalFollow, CinemachineRotationComposer, CinemachineDeoccluder, CinemachineDecollider, CinemachineImpulse .html)"
  - "Cinemachine source: https://github.com/Unity-Technologies/com.unity.cinemachine/blob/main/com.unity.cinemachine/Runtime/Core/Predictor.cs , .../Runtime/Core/InputAxis.cs"
  - "Lyra camera source (mirror): https://github.com/LeNidViolet/Lyra/tree/main/Source/LyraGame/Camera"
  - "Lyra feeler weights thread: https://forums.unrealengine.com/t/feeler-weights-seem-to-be-ignored-in-third-person-camera-avoidance-lyra/1336679"
  - "UE5 Gameplay Camera System (Epic staff posts): https://forums.unrealengine.com/t/2668478 , https://forums.unrealengine.com/t/2717870 , https://forums.unrealengine.com/t/2618820"
  - "dolly: https://github.com/h3r2tic/dolly , https://github.com/h3r2tic/dolly/blob/main/src/util.rs"
  - "Godot SpringArm3D: https://docs.godotengine.org/en/stable/classes/class_springarm3d.html"
  - "bevy_third_person_camera: https://docs.rs/crate/bevy_third_person_camera/latest"
aliases:
  - Cinemachine
  - USpringArmComponent
  - Lyra camera mode
  - Gameplay Camera System
  - dolly
  - SpringArm3D
  - virtual camera
---

# Engine camera architectures compared — Cinemachine, Unreal, Lyra, Gameplay Camera System, dolly

Contents: [Cinemachine](#unity-cinemachine-3) · [Unreal](#unreal-engine) ·
[Open source](#open-source) · [Feature table](#feature-table) ·
[Shared pipeline](#the-pipeline-they-share) · [Relevance](#relevance-to-migera)

Every production camera system splits the problem the same way:
1. **Pose the camera.** Find the target, place a pivot on it, orbit around it,
   extend an arm out to the eye, then aim.
2. **Choose and blend.** Pick which camera definition or mode is active, and
   blend between them.
3. **Correct the pose.** Collision and occlusion run after posing,
   preferably after blending.
4. **Add effects.** Shake, noise and FOV kicks go on top, last.

The systems differ in *what* they blend. Cinemachine blends the outputs of N
virtual cameras. Lyra and the Gameplay Camera System blend a push-only stack
of modes. The Unreal SpringArm and Godot SpringArm3D items below are from
memory or plain API docs where marked **[mem]**, because Epic's docs and
engine source were unreachable.

## Unity Cinemachine 3

- **Brain and virtual cameras.** The `CinemachineBrain` on the real camera picks the
  highest-priority `CinemachineCamera`; on a tie, the most recently activated one wins.
  Channels allow split screen.
  - **Blends** are per camera pair from a `CinemachineBlenderSettings` asset. The curves are
    Cut, EaseInOut, EaseIn, EaseOut, HardIn, HardOut, Linear and Custom. An exact pair
    beats the `ANY CAMERA` wildcard.
- **Pipeline per vcam:**
  - **Body** places the camera. Options:
    - `ThirdPersonFollow` is a four-pivot rig: origin → shoulder (offset, `CameraSide`
      0..1 blendable) → hand (vertical arm length) → camera (distance). It has built-in
      collision with **Damping Into / From Collision**.
    - `OrbitalFollow` uses a sphere, or **three rings** (top/middle/bottom radius and
      height, interpolated by pitch). It has radial zoom and recentring with **Wait + Time**.
  - **Aim** rotates it. `RotationComposer` keeps the target in a dead zone and a soft zone
    within hard limits, with lookahead.
  - **Noise** adds Perlin shake.
  - **Extensions** run after the pipeline: Deoccluder, Decollider, Confiner,
    Impulse Listener.
- **Deoccluder.** Keeps line of sight.
  - Strategies: PullCameraForward, PreserveCameraHeight, PreserveCameraDistance.
  - Minimum Distance From Target, Transparent Layers, Camera Radius.
  - **Minimum Occlusion Time** ignores brief occlusions.
  - **Smoothing Time** holds the camera at the nearest point.
  - **Damping vs Damping When Occluded** gives an asymmetric response.
  - Shot-quality scoring feeds `ClearShot`.
- **Decollider.** Only stops the camera clipping into geometry, plus a terrain raycast downward. Use it
  when occlusion is acceptable but clipping is not.
- **Also:** `ThirdPersonAim` raycasts forward to resolve the true aim point. `TargetGroup`
  frames several weighted targets. `StateDrivenCamera` switches cameras from animator
  states. `InputAxis` has accel/decel time, and recentring via `SmoothDamp`.
- **Damping math** (`Predictor.cs`): `initial * (1 - exp(ln(0.01) * dt / T))`, so a
  damping value *T* means "1% of the error remains after T seconds". `StableDamp` sub-steps
  at 1/1024 s, because exponential smoothing of a *moving* goal still depends on the
  frame rate.

## Unreal Engine

- **USpringArmComponent [mem].**
  - Geometry: `TargetArmLength`. `TargetOffset` (world space, at the arm origin) vs
    `SocketOffset` (arm space, at the end: the over-the-shoulder offset).
  - Collision: `bDoCollisionTest` sweeps a sphere of `ProbeSize` (≈12 cm) on
    `ProbeChannel` (Camera). The result **snaps both ways**.
  - Lag: `CameraLagSpeed` / `CameraRotationLagSpeed` via `VInterpTo`/`RInterpTo`, i.e.
    `clamp(dt·speed, 0, 1)`, which depends on the frame rate. Optional substepping
    (`CameraLagMaxTimeStep`) and `CameraLagMaxDistance` as a leash.
- **APlayerCameraManager [mem].**
  - View-target blending with Linear/Cubic/Ease functions.
  - `ViewPitchMin/Max`.
  - Priority-ordered **camera modifiers** with alpha in/out.
  - Camera shakes are themselves a modifier.
- **Lyra** (source-verified via a mirror):
  - **Push-only blend stack.** `ULyraCameraModeStack` pushes modes on top. A mode already in
    the stack keeps its current contribution, so it does not pop. Modes below a
    full-weight mode are dropped. The stack blends bottom-up. Each mode has a
    BlendTime/BlendFunction/BlendExponent and pitch limits of ±89.9°.
  - **Pivot.** Sits at eye height, compensated for crouching.
  - **Third-person mode.** `TargetOffsetCurve` is keyed by **view pitch**.
  - **Penetration avoidance.**
    - The trace starts at a **SafeLoc** inside the capsule.
    - Seven feelers: `(AdjustmentRot, WorldWeight, PawnWeight, Extent, TraceInterval)`
      - main: (0°, 1, 1, 14 cm, every frame)
      - ±16° yaw: (0.75, 0.75, ray, every 3 frames)
      - ±32° yaw: (0.5, 0.5, every 5 frames)
      - +20° pitch: (1, 1, every 4 frames)
      - −20° pitch: (0.5, 0.5, every 4 frames)
    - The main feeler is a **hard** block and snaps. The others are **soft**, predictive caps.
    - Blend in 0.1 s, out 0.15 s.
    - Below `ReportPenetrationPercent`, the pawn is told to hide.
  - A forum thread reports the feeler weights have little visible effect.
- **Gameplay Camera System (UE 5.5 experimental → 5.6+).** Camera assets hold **directors**
  (Single, Blueprint, State Tree, Priority) and **rigs** built as graphs of camera nodes:
  boom arm, offset, collision push, lens, auto-rotate. Node names are [mem]. Four layers:
  - **Base:** defaults.
  - **Main:** a push-only blend stack of rigs. Epic cites Naughty Dog's GDC talk on why a
    reordered stack causes artifacts.
  - **Global:** post-blend collision and occlusion, so a blend never passes through a wall.
  - **Visual:** additive shakes.

## Open source

- **dolly (Rust).** A `CameraRig` is a chain of drivers: `Position → YawPitch → Arm → Smooth →
  LookAt`. Smoothing is `1 - exp(-8·dt/smoothness)`. A **predictive** `Smooth`
  (negative offset scale) lags *ahead* of the parent, which gives look-ahead. No collision.
- **Godot SpringArm3D.** Casts a shape or ray with `margin`, `collision_mask` and excluded RIDs.
  Snaps, with no smoothing.
- **bevy_third_person_camera.** Orbit, zoom, shoulder offset and gamepad support; no collision.
  Version 0.4 targets Bevy 0.18.

## Feature table

| | Cinemachine 3 | UE SpringArm + Lyra | UE Gameplay Camera System | dolly |
|---|---|---|---|---|
| Blend model | N vcams, pairwise output blends | push-only mode stack | push-only rig stack + persistent layers | none |
| Collision | Deoccluder (3 strategies), Decollider | sphere sweep; Lyra weighted feelers, asymmetric | Global layer node | none |
| Asymmetric timing | yes (damping in/out, smoothing time) | Lyra yes; SpringArm no | [unverified] | — |
| Damping | exp to 1% in T, sub-stepped | linear `clamp(dt·k)`; Lyra linear blends | [unverified] | exp |
| Composer / aim | dead+soft zones, lookahead, aim raycast | none built in | framing nodes [unverified] | LookAt |
| Recentring | Wait + Time per axis | custom | auto-rotate node [mem] | custom |
| Shake | Perlin + Impulse | modifiers | Visual layer | none |

## The pipeline they share

target → **pivot** (eye height, lagged per axis) → **orbit** (yaw/pitch scalars,
clamped, recentring) → **arm** (shoulder offset, pitch-keyed curves) → *blend* →
**collision/occlusion** (sweep from a safe point, feelers, asymmetric timing) →
**aim** (composer, lookahead) → **noise/shake** (additive, after collision so it
never pushes into walls) → **post** (FOV, avatar fade).

## Relevance to migera

migera follows the **Lyra / Gameplay Camera System line**:
- one rig;
- a push-only stack;
- collision once after blending.

It also takes three pieces from elsewhere:
- Cinemachine's asymmetric timing and minimum occlusion time;
- Lyra's feeler table, as a starting point;
- Cinemachine-style "time to residual" damping, expressed as half-lives with `SpringParams`.

See [the decision](./one-rig-with-blended-layers-over-blending-virtual-cameras.md).

## Related

- [One rig with blended layers over blending virtual cameras](./one-rig-with-blended-layers-over-blending-virtual-cameras.md) — applies: the choice between these blend models.
- [Camera collision and occlusion techniques](./camera-collision-and-occlusion-techniques.md) — deeper: Deoccluder/Lyra collision in detail.
- [Camera damping is exponential, not a per-frame lerp](./camera-damping-is-exponential-not-a-per-frame-lerp.md) — deeper: why `VInterpTo` and per-frame lerps misbehave.
- [Camera and the view system](../bevy-rendering/scene-and-views/camera-and-view-system.md) — contrast: Bevy's render-side camera, which the gameplay rig only writes into.
