---
title: Gameplay camera
description: "migera's gameplay camera domain: the third-person camera design and its architecture decision, plus research on shipped games (Gothic, Journey, Witcher 3, Skyrim, Souls), engine systems (Cinemachine, Unreal, Lyra, dolly), collision and damping. Read before any camera rig, collision, lock-on or input work."
type: index
status: current
tags:
  - camera
  - prior-art
  - physics
  - correctness
updated: 2026-10-10
---

# Gameplay camera

This domain covers how the player's camera follows, frames and avoids the world. It is separate from Bevy's render-side `Camera`, which [bevy-rendering](../bevy-rendering/INDEX.md) covers. The planned code lives in `src/camera`. Its chronological log is `CAMERA_PROGRESS.md`.

## Start here

1. [One rig with blended layers](./one-rig-with-blended-layers-over-blending-virtual-cameras.md): the architecture choice.
2. [Third-person camera design](./third-person-camera-design.md): the pipeline built on it.
3. [Camera collision and occlusion techniques](./camera-collision-and-occlusion-techniques.md): the hardest part.

## Key facts

- Collision snaps in and eases out after a hold. Occlusion waits a minimum time before it acts at all. Symmetric timing is a visible defect in Gothic, UE SpringArm and Godot ([collision](./camera-collision-and-occlusion-techniques.md)).
- Sweep from a safe point inside the character. Use a sphere at least the size of the near-plane half-diagonal, about 0.13 m at Bevy's defaults ([collision](./camera-collision-and-occlusion-techniques.md)).
- Characters and thin props must not block the camera. Otherwise it bounces off enemy legs, as in DS3 and AC Syndicate ([shipped behaviours](./shipped-action-game-camera-behaviours.md)).
- Store the camera as orbit scalars. Derive distance and FOV from pitch "like gears". Give each degree of freedom one owner ([Nesky digest](./fifty-camera-mistakes-nesky-digest.md)).
- Smoothing must be closed-form exponential or a critically damped spring. `lerp(k·dt)` (Gothic `veloTrans`, UE `VInterpTo`) changes feel with frame rate ([damping](./camera-damping-is-exponential-not-a-per-frame-lerp.md)).
- Modes are layers on one rig. Collision runs once, after blending ([decision](./one-rig-with-blended-layers-over-blending-virtual-cameras.md)).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [The camera is one rig with blended layers, not blended virtual cameras](./one-rig-with-blended-layers-over-blending-virtual-cameras.md) | Layers blend parameters, anchors and goals. Collision runs after the blend. Scripted shots use an override layer. | Before adding a camera mode, a blend, or a lock-on/dialog driver |
| [Third-person camera design](./third-person-camera-design.md) | The pipeline, components, scheduling, collision stages, lock-on, profiles, trace replay and test strategy | Before building or extending `src/camera` |
| [Camera collision and occlusion techniques](./camera-collision-and-occlusion-techniques.md) | Safe pivot, probe size, timing, feelers, hills, ceilings, layers, fades and the fallback | Before building or tuning camera collision, or when the camera clips or pumps |
| [Camera damping is exponential, not a per-frame lerp](./camera-damping-is-exponential-not-a-per-frame-lerp.md) | The only allowed smoothing forms, unit conversions, moving goals and angle damping | Before writing any camera smoothing, or when feel changes with frame rate |
| [Gothic's ZenGin camera](./gothic-zengin-camera-modes-and-collision.md) | Per-mode parameter tables, the 9-ray grid and top-down fallback, and their weaknesses | When choosing mode parameters or a collision fallback |
| [Fifty game camera mistakes (Nesky) — digest](./fifty-camera-mistakes-nesky-digest.md) | Nesky's 50 practitioner rules, grouped | Before designing camera degrees of freedom, auto-recentre or input curves |
| [Engine camera architectures compared](./engine-camera-architectures-compared.md) | Cinemachine, Unreal, Lyra, the Gameplay Camera System, dolly and Godot, with a feature table | When comparing blend models or looking for a reference implementation |
| [Shipped action-game camera behaviours](./shipped-action-game-camera-behaviours.md) | Per-mode distances, combat framing, lock-on, named anti-patterns, trauma shake and the TLOU2 option set | When choosing modes, lock-on rules, shake or player options |

## See also

- [Camera and the view system](../bevy-rendering/scene-and-views/camera-and-view-system.md): Bevy's render-side camera, which the rig writes into.
- [The rational exp approximation in spring code diverges](../character-animation/animation-core/spring-exp-approximation-diverges.md): the spring code the camera reuses.
