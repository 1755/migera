---
title: Gothic's ZenGin camera — per-mode parameter sets, a near-plane ray grid, and a top-down fallback
description: "Case study of Gothic 1/2's camera from its shipped scripts (CCamSys, CamInst.d) and OpenGothic's reimplementation: range/elevation/azimuth modes, veloTrans/veloRot follow, a 9-ray near-plane collision grid with a top-down fallback, and its weaknesses. Read before choosing mode parameters or collision fallbacks."
type: research
status: current
tags:
  - camera
  - prior-art
  - case-study
  - correctness
updated: 2026-10-10
sources:
  - "Camera.d (CCamSys class, Gothic 2 Addon scripts): https://raw.githubusercontent.com/VaanaCZ/gothic-2-addon-scripts/2.7-EN/_work/Data/Scripts/System/_intern/Camera.d"
  - "CamInst.d (per-mode instances): https://raw.githubusercontent.com/VaanaCZ/gothic-2-addon-scripts/2.7-EN/_work/Data/Scripts/System/Camera/CamInst.d"
  - "OpenGothic common/camera.cpp (reverse-engineered reimplementation): https://raw.githubusercontent.com/Try/OpenGothic/master/common/camera.cpp"
  - "Gothic 1 Remake (UE5, 2026-06-05): https://en.wikipedia.org/wiki/Gothic_1_Remake"
  - "Remake player threads: https://steamcommunity.com/app/1297900/discussions/0/569247980380494775/ , https://steamcommunity.com/app/1297900/discussions/0/563658656215998016/"
aliases:
  - CCamSys
  - CamInst.d
  - CAMERA.DAT
  - veloTrans
  - CamModNormal
---

# Gothic's ZenGin camera — per-mode parameter sets, a near-plane ray grid, and a top-down fallback

Gothic 1/2 drive the camera from **one rig and ~30 named parameter sets**
(`CamMod*`), each expressed in orbit terms: range, elevation, azimuth, plus
offsets and two follow speeds. Collision is a grid of rays to the
near-plane corners that shortens the range. When even that fails, the camera
jumps to a near-overhead view. This is the clearest shipped example of "a mode is
a parameter set on one rig", and of what goes wrong without asymmetric
collision timing and a follow leash.

## The parameter schema (`CCamSys`, compiled into `CAMERA.DAT`)

Fields in `_intern/Camera.d`:
- **Range:** `bestRange` / `minRange` / `maxRange`, in metres.
- **Angles:** `best/min/maxElevation`, `best/min/maxAzimuth` and `best/min/maxRotZ`, in degrees from −180 to 180. Elevation is positive when the camera is above.
- **Offsets:** `rotOffsetX/Y/Z` (a fixed rotation added to the look direction) and `targetOffsetX/Y/Z` (the pivot offset from the NPC).
- **Speeds:** `veloTrans` ("velocity while easing to best position") and `veloRot` ("velocity while rotating to best orientation").
- **Flags:** `translate`, `rotate` and `collision`.

The prototype defaults are: range 2.0 (min 1.99, max 4.01), elevation 0–89°, azimuth ±90°,
`rotOffsetX` 20, `veloTrans` 40, `veloRot` 2, collision on.

## Mode instances (`CamInst.d`)

The script header warns: *"minRange besser nicht unter 1.5"*, meaning "better not
below 1.5 m".

| Mode | best range | range limits | best elevation | notes |
|---|---|---|---|---|
| `CamModNormal` | 3.0 m | 2–10 | 30° | `rotOffsetX` 23; commented "Tombraider Style" |
| `CamModMelee` / `Ranged` / `Magic` | 2.5 m | 1.4–10 (magic 6) | 35° | combat pulls in and up |
| `CamModSwim` | 3.0 m | | 20° | elevation clamped 10–45° |
| `CamModDive` | 3.0 m | | −20° | looks up from below; `rotate` 0; `veloTrans` 20 |
| `CamModFall` | | | 60° | `veloTrans` 10, a slow, heavy follow |
| `CamModDeath` | | | 80° | azimuth 180, so it looks at the face |
| `CamModDialog` | 3.0 m | | | azimuth 45 |
| `CamModFirstPerson` | 2.0 m | | | `veloRot` 10000, effectively instant |

Interactable objects ("mobsi": doors, ladders, beds) use the same mechanism as
context cameras. For example, door-front is range 1.25 m, elevation 40°, azimuth 45°.

## How the engine applies them (OpenGothic's reimplementation)

OpenGothic is a reverse-engineered reimplementation, not ZenGin source. Its
behaviour matches the shipped game closely enough that its comments name
vanilla quirks.

- **Follow:** `origin += (dest - origin) * min(1, 0.25 * veloTrans * dt)`. At
  `veloTrans` 40 this is ≈ 10/s, a time constant of ≈ 0.1 s. Note the `min(1, k·dt)`
  form: it is a per-frame lerp and depends on the frame rate (see
  [camera damping](./camera-damping-is-exponential-not-a-per-frame-lerp.md)).
- **Rotation offset:** slerps at `veloRot · dt`.
- **Pivot:** follows the target with its own leash-like spring, so it lags the NPC.
- **Collision:** a **3×3 grid of 9 rays** from the pivot toward the
  near-plane points, with **25 cm padding**. The range is clamped to the nearest hit.
  The rays start from the *offset target*, not the NPC, so the start point is
  above the body.
- **Fallback:** *"with range < 80, camera gradually moves up in vanilla"*.
  When collision forces the range below 0.8 m, the camera switches to range
  1.5 m at 80° elevation, almost overhead.
- **Elevation clamp:** −60…+85°. Vanilla appears to ignore `minElevation` 0.
- **Collision timing:** the collided position goes through the same `veloTrans` lerp, so
  pulling in and easing out happen at the same speed.

## Weaknesses that show in play

- **Symmetric collision response.** The camera eases into walls as slowly as it eases out,
  so a wall briefly clips into view. Modern rigs snap in and ease out; see
  [collision techniques](./camera-collision-and-occlusion-techniques.md).
- **It zooms out while running.** A WorldOfPlayers thread is titled "prevent the camera
  zooming out when running". The page was blocked, so this comes from the title only.
  The lagged pivot plus a fixed range means the camera trails further back the
  faster the hero moves, because there is no hard leash on the lag.
- **Gothic 1 Remake (Alkimia, UE5, 2026).** Players say the default
  off-centre camera sits too far back ("Witcher style"), compared with the original's
  camera "right behind the hero". They also report that the camera-mode setting does not
  apply in combat. A centred option exists. Mods add FOV, classic-camera and
  first-person modes; their pages returned 403, so there are no numbers.

## Relevance to migera

- **Seed values:** exploration ≈ 3 m with elevation 15–30°; combat pulls in to
  ≈ 2.5 m and raises elevation; a minimum range of ≈ 1.5 m before falling back.
  These belong in the RON profile, not in code.
- **Copy:** one rig with named parameter sets per mode and per context object;
  starting the collision sweep from an offset pivot, never the feet; padding to the
  near plane; a last-resort high view when space runs out.
- **Fix:** a frame-rate-independent follow, asymmetric collision timing, and a leash on
  pivot lag.
- **Offer** a centred shoulder offset alongside an over-the-shoulder one; the Remake
  complaint shows players disagree on which is right.

## Related

- [Third-person camera design](./third-person-camera-design.md) — applies: modes as parameter layers, the leash, and the fallback come from here.
- [Camera collision and occlusion techniques](./camera-collision-and-occlusion-techniques.md) — deeper: the asymmetric response Gothic lacks.
- [Camera damping is exponential, not a per-frame lerp](./camera-damping-is-exponential-not-a-per-frame-lerp.md) — contrast: why `min(1, k·dt)` misbehaves.
- [Shipped action-game camera behaviours](./shipped-action-game-camera-behaviours.md) — contrast: how other games did per-mode distances.
