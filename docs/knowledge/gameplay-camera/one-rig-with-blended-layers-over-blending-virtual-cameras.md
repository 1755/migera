---
title: The camera is one rig with blended layers, not blended virtual cameras
description: "migera's third-person camera is one rig per camera. Modes are layers blending rig parameters in a push-only stack; anchors hand off by inertialization; yaw/pitch goals own axes by priority; collision runs once after the blend. Cinemachine-style pose blending lost. Read before adding a camera mode or blend."
type: decision
status: current
tags:
  - camera
  - ecs
  - correctness
  - integration
updated: 2026-10-10
code:
  - src/camera/stack.rs
  - src/camera/anchor.rs
  - src/camera/orbit.rs
sources:
  - "Lyra ULyraCameraModeStack: https://github.com/LeNidViolet/Lyra/tree/main/Source/LyraGame/Camera"
  - "UE5 Gameplay Camera System layers (Epic staff): https://forums.unrealengine.com/t/2668478"
  - "Cinemachine blending: https://docs.unity3d.com/Packages/com.unity.cinemachine@3.1/manual/CinemachineBlending.html"
  - "Gothic CamInst.d mode sets: https://raw.githubusercontent.com/VaanaCZ/gothic-2-addon-scripts/2.7-EN/_work/Data/Scripts/System/Camera/CamInst.d"
aliases:
  - camera layer stack
  - ModeStack
  - camera mode blending
---

# The camera is one rig with blended layers, not blended virtual cameras

Each gameplay camera is **one rig**: one pivot, one orbit, one collision pass.
A camera mode (exploration, combat, aim, swim, mount, dialog) is a **layer** in a
push-only stack that blends rig **parameters** by weight. Two other inputs
change in their own ways:
- an **anchor** (what the pivot follows) is handed off by inertialization;
- a yaw/pitch **goal** (lock-on, dialog, assist) owns its axis by priority.

Collision runs once, on the blended result. Only scripted shots blend in output
space, through a separate override layer.

## Context

An action-RPG camera moves between many behaviours, often several times a second:
- exploration → combat, combat → lock-on, aim in/out;
- swimming, climbing, mounting;
- dialog, death and ragdoll.

Every transition must keep the eye continuous in position and velocity. It must not cut
through the character or a wall, and it must not let two behaviours fight over the same
degree of freedom. migera had no camera when this was decided (2026-10-10); see
[engine architectures compared](./engine-camera-architectures-compared.md) for the
designs reviewed.

## Decision

- **One rig, a stack of layers.** `Layer { id, params, progress, blend }`, with weight
  `smoothstep(progress)` (`src/camera/stack.rs`). The stack is push-only:
  - pushing a mode that is already present keeps its current weight (no pop);
  - the top layer's weight ramps up with a C1 curve;
  - layers under a full-weight top are dropped;
  - the stack is evaluated bottom-up.
- **Parameters** (distance/height/FOV pitch curves, shoulder side, pitch limits, recentre
  settings, collision policy) are interpolated. Because distance and angles are blended
  as *orbit scalars*, the blended eye stays on an orbit around the pivot. It never travels
  the chord between two eye positions, which can pass through the character.
- **Anchors are handed off, not blended.** An anchor switch (mounting, the target dying
  into a ragdoll, a dialog midpoint) restarts the follow on the new goal, at its steady
  trail. The old pivot's position and velocity, relative to that, become an offset that
  decays on its own softer spring. This keeps velocity continuous and sets the hand-off's
  pace independently of the stiff follow spring: a 3 m switch accelerates the pivot at
  under 50 m/s², against about 1200 m/s² if it rode the follow spring
  (`an_anchor_switch_is_velocity_continuous`).
  - Blending anchors as weighted world points was the first design. It was dropped
    because it needs every layer's anchor resolved every frame, and the hand-off gives
    the same continuity with one goal.
- **Goals** (lock-on, dialog, aim assist, recentre) are supplied per frame, because their
  targets move. They drive yaw and pitch toward a target with a half-life. **Each degree
  of freedom has one owner per frame**: the highest-priority goal. Player input under a
  goal becomes an offset that decays once the player lets go. When the goal ends, the
  offset folds into the base angle, so nothing jumps (Nesky: intent wins, no fighting
  forces).
- **Collision runs once, after the blend.** This is the Gameplay Camera System's "Global"
  layer. A blend between two collision-free poses can still pass through a wall, so
  correcting each mode before blending is not enough.
- **Scripted shots** (cinematics, fixed dialog shots) use an output-space
  `CameraOverride` with blend in and out. This is the only place eye poses are blended
  directly.
- **Modes are data.** Each mode is a named `ModeId` in the RON profile, with inheritance
  and selection rules. Modes are not a closed Rust enum, so a game adds "stealth" or
  "mounted" without touching the camera crate.

## Alternatives considered

- **N virtual cameras with pairwise output blending (Cinemachine).**
  - Each camera owns a full pipeline, and the Brain blends their output poses.
  - Strengths: flexible, designer-friendly, good for many authored shots.
  - Why it lost:
    - Blending poses interpolates eye positions along a chord, which can cut through the
      character during a swing between two shoulders.
    - Each camera runs its own collision, so the blended result is not collision-checked
      unless an extra pass is added.
    - Lock-on and recentring become separate cameras whose blend fights player input.
  - What stayed: the override layer keeps this model for authored shots.
- **A spring arm per mode with view-target blending (Unreal's classic setup).** It has
  the same chord and collision problems, plus frame-rate-dependent lag (`VInterpTo`) and
  symmetric collision snapping.
- **A general constraint solver.** Rejected per Nesky #50. Constraints fight, and tuning
  becomes opaque.

## Consequences

- Every mode must be expressible as parameters, an anchor or a goal on the shared rig. A
  shot that cannot, such as a fixed security-camera angle, goes through the override.
- **Reordering the stack is forbidden.** Re-pushing a mode moves it to the top while
  keeping its weight. This avoids the artifacts Epic attributes to reordered stacks,
  citing Naughty Dog's GDC talk.
- **Testable properties:**
  - eye position and velocity are continuous across any push;
  - the blended eye stays on the orbit band;
  - an anchor switch is velocity-continuous.
- **One owner per degree of freedom** means goal arbitration needs an explicit priority
  order in the profile.

## Revisit when

- Many authored cinematic shots are needed, and the override layer grows into its own
  camera system. At that point consider a Cinemachine-style director above the rig.
- Split-screen or picture-in-picture needs cameras that share layers. Today each camera
  entity owns its stack.

## Related

- [Third-person camera design](./third-person-camera-design.md) — applies: the full pipeline built around this choice.
- [Engine camera architectures compared](./engine-camera-architectures-compared.md) — contrast: the blend models that lost, in detail.
- [Gothic's ZenGin camera](./gothic-zengin-camera-modes-and-collision.md) — example: a shipped game with modes as parameter sets on one rig.
- [Fifty camera mistakes digest](./fifty-camera-mistakes-nesky-digest.md) — prerequisite: the one-owner-per-DOF and no-solver rules.
